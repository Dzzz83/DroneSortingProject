"""ROS adapter for the local depth map; never publishes motion commands."""
import json
import math
import threading
import time
from dataclasses import asdict

import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32, Header, String

from drone_sorting.interfaces import ros_topics as topics
from drone_sorting.mapping.depth_map import DepthObservation, LocalDepthMap, MapSettings
from drone_sorting.mapping.local_planner import LocalPlanner, VehicleEnvelope
from drone_sorting.perception.camera_geometry import transform_point


def decode_observation(depth, info, camera, drone, now, maximum_age):
    messages = (depth, info, camera, drone)
    if any(message.header.stamp != depth.header.stamp for message in messages):
        raise ValueError("acquisition_timestamps_differ")
    if depth.header.frame_id != "forward_camera_optical" or info.header.frame_id != depth.header.frame_id:
        raise ValueError("incorrect_optical_frame")
    if camera.header.frame_id != "warehouse" or drone.header.frame_id != "warehouse":
        raise ValueError("incorrect_pose_frame")
    stamp = depth.header.stamp.to_sec()
    if not 0 <= now-stamp <= maximum_age:
        raise ValueError("stale_or_future_observation")
    if (depth.encoding != "32FC1" or depth.is_bigendian not in (0,1)
            or depth.width == 0 or depth.height == 0 or depth.step != depth.width*4
            or len(depth.data) != depth.step*depth.height):
        raise ValueError("invalid_depth_layout")
    if (info.width != depth.width or info.height != depth.height
            or info.distortion_model != "plumb_bob" or any(value != 0 for value in info.D)):
        raise ValueError("invalid_camera_calibration")
    p, q = camera.pose.position, camera.pose.orientation
    body, rotation = drone.pose.position, drone.pose.orientation
    position, quaternion = (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)
    root = (body.x,body.y,body.z)
    transform_point((0,0,0), root, (rotation.x,rotation.y,rotation.z,rotation.w))
    if math.dist(position, (body.x,body.y,body.z-.5)) > .001:
        raise ValueError("unexpected_camera_mount")
    dtype = ">f4" if depth.is_bigendian else "<f4"
    image = np.frombuffer(bytes(depth.data), dtype=dtype).reshape(depth.height,depth.width)
    return DepthObservation(stamp, image, tuple(info.K), position, quaternion, root)


class DepthMappingNode:
    def __init__(self):
        defaults = asdict(MapSettings())
        settings = MapSettings(**{name:rospy.get_param("~"+name,value) for name,value in defaults.items()})
        self.maximum_age = float(rospy.get_param("~maximum_observation_age", .5))
        self.update_rate = float(rospy.get_param("~update_rate", 4.))
        if not 0 < self.maximum_age <= settings.maximum_age or not math.isfinite(self.update_rate) or self.update_rate <= 0:
            raise ValueError("Invalid observation age or map update rate")
        heading = rospy.get_param("~camera_heading", [0.,1.])
        if len(heading) != 2 or not all(math.isfinite(value) for value in heading) or math.hypot(*heading) < 1e-6:
            raise ValueError("Camera heading requires finite nonzero ROS X/Y")
        length = math.hypot(*heading)
        self.heading = Vector3(heading[0]/length,heading[1]/length,0)
        self.map = LocalDepthMap(settings)
        self.lock = threading.RLock()
        self.pending = None
        self.stopping = False
        self.reset_pending = False
        self.reset_barrier = -math.inf
        self.reset_sequence = None
        self.range_max = None
        self.last_stamp = None
        self.latest_position = None
        self.local_goal = None
        dimensions = rospy.get_param("~planner_envelope", None)
        if dimensions is not None and (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 4):
            raise ValueError("planner_envelope must contain four measured extents")
        self.planner = (LocalPlanner(self.map, VehicleEnvelope(*dimensions),
                                     int(rospy.get_param("~planner_max_expansions", 4000)))
                        if dimensions is not None else None)
        self.problem = "waiting_for_synchronized_observation"
        self.processing_ms = None
        self.accepted = self.rejected = self.superseded = 0
        self.enabled_publisher = rospy.Publisher(topics.FORWARD_CAMERA_ENABLED, Bool, queue_size=1, latch=True)
        self.heading_publisher = rospy.Publisher(topics.FORWARD_CAMERA_HEADING, Vector3, queue_size=1, latch=True)
        self.occupied_publisher = rospy.Publisher(topics.LOCAL_MAP_OCCUPIED, PointCloud2, queue_size=1)
        self.free_publisher = rospy.Publisher(topics.LOCAL_MAP_FREE, PointCloud2, queue_size=1)
        self.status_publisher = rospy.Publisher(topics.LOCAL_MAP_STATUS, String, queue_size=1)
        self.plan_path_publisher = rospy.Publisher(topics.LOCAL_PLAN_PATH, Path, queue_size=1)
        self.plan_status_publisher = rospy.Publisher(topics.LOCAL_PLAN_STATUS, String, queue_size=1)
        self.goal_subscriber = rospy.Subscriber(topics.LOCAL_PLAN_GOAL, Point, self.receive_goal, queue_size=1)
        self.range_subscriber = rospy.Subscriber(topics.FORWARD_RANGE_MAX, Float32, self.receive_range, queue_size=1)
        self.reset_subscriber = rospy.Subscriber(topics.COURSE_OBSERVATIONS, String, self.receive_course, queue_size=1)
        streams = [(topics.FORWARD_DEPTH_IMAGE,Image), (topics.FORWARD_CAMERA_INFO,CameraInfo),
                   (topics.FORWARD_CAMERA_POSE,PoseStamped), (topics.FORWARD_DRONE_POSE,PoseStamped)]
        self.subscribers = [message_filters.Subscriber(topic,kind,queue_size=1,buff_size=2*1024*1024)
                            for topic,kind in streams]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers,queue_size=5)
        self.synchronizer.registerCallback(self.receive_bundle)
        self.request_timer = rospy.Timer(rospy.Duration(1), self.request_camera)
        rospy.on_shutdown(self.shutdown)
        self.request_camera()

    def request_camera(self, _event=None):
        # Repeat through ROS-TCP subscription/reconnection timing, not just once.
        with self.lock:
            if self.stopping:
                return
            self.heading_publisher.publish(self.heading)
            self.enabled_publisher.publish(Bool(True))

    def receive_range(self, message):
        with self.lock:
            self.range_max = message.data
            if self.range_max != 15:
                self.problem = "missing_or_invalid_range_declaration"

    def receive_course(self, message):
        try:
            sequence = json.loads(message.data)["reset_sequence"]
            if not isinstance(sequence,int) or isinstance(sequence,bool) or sequence < 0:
                return
        except (ValueError,KeyError,TypeError):
            return
        with self.lock:
            if self.reset_sequence is not None and sequence != self.reset_sequence:
                self.reset_pending = True
                self.reset_barrier = rospy.get_time()
                self.pending = None
                self.latest_position = None
                self.local_goal = None
            self.reset_sequence = sequence

    def receive_bundle(self, *messages):
        with self.lock:
            if self.pending is not None:
                if messages[0].header.stamp <= self.pending[0].header.stamp:
                    self.rejected += 1
                    return
                self.superseded += 1
            self.pending = messages

    def receive_goal(self, message):
        goal = (message.x, message.y, message.z)
        if not all(math.isfinite(value) for value in goal):
            return
        with self.lock:
            self.local_goal = goal

    def process_once(self, now=None):
        with self.lock:
            now = rospy.get_time() if now is None else now
            if self.reset_pending:
                self.map.reset()
                self.last_stamp = None
                self.latest_position = None
                self.problem = "course_reset"
                self.reset_pending = False
            bundle, self.pending = self.pending, None
            if bundle is not None:
                started = time.perf_counter()
                try:
                    if self.range_max != 15:
                        raise ValueError("missing_or_invalid_range_declaration")
                    observation = decode_observation(*bundle, now, self.maximum_age)
                    if observation.stamp <= self.reset_barrier:
                        raise ValueError("observation_precedes_reset")
                    # A long stream gap starts a new observation history. This
                    # also avoids carrying an earlier Play session into recovery.
                    if self.last_stamp is not None and observation.stamp-self.last_stamp > self.maximum_age:
                        self.map.reset()
                        self.latest_position = None
                    self.map.integrate(observation, now)
                    self.latest_position = observation.drone_position
                    self.last_stamp = observation.stamp
                    self.problem = None
                    self.accepted += 1
                except (ValueError,TypeError,IndexError) as error:
                    self.problem = str(error)
                    self.rejected += 1
                self.processing_ms = (time.perf_counter()-started)*1000
            self.publish(now)

    def publish(self, now):
        # Snapshot age is evaluated even when no observations arrive, so both
        # clouds become empty after expiry rather than displaying stale cells.
        points = {"occupied":[],"free":[]}
        resolution = self.map.settings.resolution
        for cell,state,_age in self.map.snapshot(now):
            points[state].append(tuple((index+.5)*resolution for index in cell))
        header = Header(stamp=rospy.Time.from_sec(now),frame_id="warehouse")
        self.occupied_publisher.publish(point_cloud2.create_cloud_xyz32(header,points["occupied"]))
        self.free_publisher.publish(point_cloud2.create_cloud_xyz32(header,points["free"]))
        age = None if self.last_stamp is None else now-self.last_stamp
        reason = self.problem
        if age is not None and not 0 <= age <= self.maximum_age:
            reason = "stale_observation"
        status = {"stamp":now,"frame_id":"warehouse","state":"ACTIVE" if reason is None else "WAITING",
                  "reason":reason,"observation_age":age,"processing_ms":self.processing_ms,
                  "accepted":self.accepted,"rejected":self.rejected,"superseded":self.superseded,
                  "occupied_cells":len(points["occupied"]),"free_cells":len(points["free"]),
                  "reset_sequence":self.reset_sequence}
        self.status_publisher.publish(String(json.dumps(status,allow_nan=False)))
        self.publish_plan(now, status)

    def publish_plan(self, now, mapping_status):
        # Same in-memory map as depth processing; no cloud reconstruction.
        header = Header(stamp=rospy.Time.from_sec(now), frame_id="warehouse")
        route = Path(header=header)
        state, reason, expanded = "HOLD", "envelope_not_configured", 0
        if self.planner is not None:
            if self.local_goal is None:
                reason = "no_local_goal"
            elif self.latest_position is None:
                reason = "no_current_pose"
            elif mapping_status["state"] != "ACTIVE":
                reason = "map_" + str(mapping_status["reason"] or "unavailable")
            else:
                result = self.planner.plan(self.latest_position, self.local_goal, now)
                state, reason, expanded = result.state, result.reason, result.expanded
                if state == "READY":
                    for waypoint in result.path:
                        pose = PoseStamped(header=header)
                        pose.pose.position = Point(*waypoint)
                        pose.pose.orientation.w = 1.0
                        route.poses.append(pose)
        self.plan_path_publisher.publish(route)
        details = {"stamp":now, "frame_id":"warehouse", "state":state,
                   "reason":reason, "waypoints":len(route.poses),
                   "expanded":expanded, "reset_sequence":self.reset_sequence}
        self.plan_status_publisher.publish(String(json.dumps(details,allow_nan=False)))

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.process_once()
            rate.sleep()

    def shutdown(self):
        self.request_timer.shutdown()
        with self.lock:
            self.stopping = True
            self.enabled_publisher.publish(Bool(False))
