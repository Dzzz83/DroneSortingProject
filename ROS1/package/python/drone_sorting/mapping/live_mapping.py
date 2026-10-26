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
from drone_sorting.mapping.local_goals import LocalGoalPlanner
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
        self.default_heading = self.heading
        self.volume_scan = rospy.get_param("~volume_scan", False)
        self.scan_index = 0
        self.scan_directions = ((0., 1., 0.), (1., 0., 0.), (0., -1., 0.),
                                (-1., 0., 0.), (0., 0., 1.), (0., 0., -1.))
        self.scan_heading_received = None
        self.map = LocalDepthMap(settings)
        self.lock = threading.RLock()
        # Camera receipt must never wait for map integration or planning. Only
        # the worker touches the map; this separate lock protects its inbox.
        self.pending_lock = threading.Lock()
        self.pending = None
        self.pending_timing = None
        self.timings = {}
        self.stopping = False
        self.reset_pending = False
        self.reset_barrier = -math.inf
        self.reset_sequence = None
        self.range_max = None
        self.last_stamp = None
        self.last_capture_sequence = None
        self.latest_position = None
        self.latest_camera = None
        self.local_goal = None
        dimensions = rospy.get_param("~planner_envelope", None)
        if dimensions is not None and (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 4):
            raise ValueError("planner_envelope must contain four measured extents")
        self.planner = (LocalPlanner(self.map, VehicleEnvelope(*dimensions),
                                     int(rospy.get_param("~planner_max_expansions", 4000)),
                                     allow_vertical=rospy.get_param("~planner_allow_vertical", False))
                        if dimensions is not None else None)
        self.goal_selector = (LocalGoalPlanner(self.planner)
                              if self.planner and rospy.get_param('~select_local_goals', False) else None)
        self.unloaded_envelope = self.planner.envelope if self.planner else None
        carried = rospy.get_param("~planner_carried_envelope", None)
        if carried is not None and (not isinstance(carried, (list, tuple)) or len(carried) != 4):
            raise ValueError("planner_carried_envelope must contain four measured extents")
        self.carried_envelope = VehicleEnvelope(*carried) if carried is not None else None
        if (self.unloaded_envelope and self.carried_envelope and
                any(a < b for a, b in zip(vars(self.carried_envelope).values(),
                                          vars(self.unloaded_envelope).values()))):
            raise ValueError("Carried envelope must contain the unloaded envelope")
        self.package_attached = False
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
        self.scan_subscriber = rospy.Subscriber(topics.LOCAL_PLAN_SCAN_HEADING, Vector3, self.receive_scan_heading, queue_size=1)
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
            if self.scan_heading_received is not None and rospy.get_time()-self.scan_heading_received > 1.0:
                self.heading = self.default_heading
                self.scan_heading_received = None
            self.heading_publisher.publish(self.heading)
            self.enabled_publisher.publish(Bool(True))

    def receive_scan_heading(self, message):
        # Full-volume mode advances only after acquiring the requested view.
        # Controller timing must not skip views when capture/transport is slow.
        if self.volume_scan:
            return
        values = (message.x, message.y)
        if not all(math.isfinite(value) for value in values):
            return
        magnitude = math.hypot(*values)
        if magnitude < 1e-6:
            return
        with self.lock:
            self.heading = Vector3(values[0]/magnitude, values[1]/magnitude, 0)
            self.scan_heading_received = rospy.get_time()
            self.heading_publisher.publish(self.heading)

    def receive_range(self, message):
        with self.lock:
            self.range_max = message.data
            if self.range_max != 15:
                self.problem = "missing_or_invalid_range_declaration"

    def receive_course(self, message):
        try:
            payload = json.loads(message.data)
            sequence = payload["reset_sequence"]
            if not isinstance(sequence,int) or isinstance(sequence,bool) or sequence < 0:
                return
        except (ValueError,KeyError,TypeError):
            return
        with self.lock:
            attached = payload.get("package_attached")
            if isinstance(attached, bool):
                self.package_attached = attached
                if self.planner is not None:
                    self.planner.envelope = (self.carried_envelope if attached and self.carried_envelope
                                             else self.unloaded_envelope)
            if self.reset_sequence is not None and sequence != self.reset_sequence:
                self.reset_pending = True
                self.reset_barrier = rospy.get_time()
                with self.pending_lock:
                    self.pending = None
                    self.pending_timing = None
                self.latest_position = None
                self.latest_camera = None
                self.local_goal = None
            self.reset_sequence = sequence

    def receive_bundle(self, *messages):
        received_stamp = rospy.get_time()
        received = time.perf_counter()
        with self.pending_lock:
            if self.pending is not None:
                if messages[0].header.stamp <= self.pending[0].header.stamp:
                    self.rejected += 1
                    return
                self.superseded += 1
            self.pending = messages
            queued = time.perf_counter()
            self.pending_timing = (received_stamp, queued, (queued-received)*1000)

    def receive_goal(self, message):
        goal = (message.x, message.y, message.z)
        if not all(math.isfinite(value) for value in goal):
            return
        with self.lock:
            self.local_goal = goal

    def process_once(self, now=None):
        # Explicit now freezes the clock for replay/tests. Live checks take a
        # fresh time after integration and again before publishing approval.
        clock = rospy.get_time if now is None else lambda: now
        with self.lock:
            self.timings = {}
            now = clock()
            if self.reset_pending:
                self.map.reset()
                if self.goal_selector:
                    self.goal_selector.reset()
                self.last_stamp = None
                self.last_capture_sequence = None
                self.latest_position = None
                self.latest_camera = None
                self.problem = "course_reset"
                self.reset_pending = False
            with self.pending_lock:
                bundle, self.pending = self.pending, None
                pending_timing, self.pending_timing = self.pending_timing, None
            if bundle is not None:
                started = time.perf_counter()
                if pending_timing is not None:
                    received_stamp, queued, lock_wait = pending_timing
                    self.timings.update({
                        "camera_to_bundle_ms": (received_stamp-bundle[0].header.stamp.to_sec())*1000,
                        "callback_lock_wait_ms": lock_wait,
                        "pending_wait_ms": (started-queued)*1000})
                try:
                    if self.range_max != 15:
                        raise ValueError("missing_or_invalid_range_declaration")
                    observation = decode_observation(*bundle, now, self.maximum_age)
                    if observation.stamp <= self.reset_barrier:
                        raise ValueError("observation_precedes_reset")
                    # A long stream gap starts a new observation history. This
                    # also avoids carrying an earlier Play session into recovery.
                    gap_limit = self.map.settings.maximum_age if self.volume_scan else self.maximum_age
                    sequence = bundle[0].header.seq
                    new_capture_session = (self.last_capture_sequence is not None
                                           and sequence < self.last_capture_sequence)
                    if new_capture_session or (self.last_stamp is not None
                                                and observation.stamp-self.last_stamp > gap_limit):
                        self.map.reset()
                        if self.goal_selector:
                            self.goal_selector.reset()
                        self.latest_position = None
                        self.latest_camera = None
                    integration_started = time.perf_counter()
                    self.map.integrate(observation, now)
                    self.timings["integration_ms"] = (time.perf_counter()-integration_started)*1000
                    self.latest_position = observation.drone_position
                    forward = transform_point((0, 0, 1), (0, 0, 0), observation.quaternion)
                    self.latest_camera = (observation.position, forward)
                    if self.volume_scan:
                        expected = self.scan_directions[self.scan_index]
                        if sum(a*b for a, b in zip(forward, expected)) > .99:
                            self.scan_index = (self.scan_index+1) % len(self.scan_directions)
                        self.heading = Vector3(*self.scan_directions[self.scan_index])
                        self.heading_publisher.publish(self.heading)
                    self.last_stamp = observation.stamp
                    self.last_capture_sequence = sequence
                    self.problem = None
                    self.accepted += 1
                except (ValueError,TypeError,IndexError) as error:
                    self.problem = str(error)
                    with self.pending_lock:
                        self.rejected += 1
                self.processing_ms = (time.perf_counter()-started)*1000
            self.publish(clock(), clock)

    def publish(self, now, clock=None):
        # Snapshot age is evaluated even when no observations arrive, so both
        # clouds become empty after expiry rather than displaying stale cells.
        points = {"occupied":[],"free":[]}
        counts = {"occupied": 0, "free": 0}
        cloud_requested = (self.occupied_publisher.get_num_connections()
                           or self.free_publisher.get_num_connections())
        snapshot_started = time.perf_counter()
        resolution = self.map.settings.resolution
        evidence = self.map.evidence(now)
        for cell,state in evidence.items():
            counts[state] += 1
            if cloud_requested:
                points[state].append(tuple((index+.5)*resolution for index in cell))
        self.timings["snapshot_ms"] = (time.perf_counter()-snapshot_started)*1000
        header = Header(stamp=rospy.Time.from_sec(now),frame_id="warehouse")
        if cloud_requested:
            self.occupied_publisher.publish(point_cloud2.create_cloud_xyz32(header,points["occupied"]))
            self.free_publisher.publish(point_cloud2.create_cloud_xyz32(header,points["free"]))
        age = None if self.last_stamp is None else now-self.last_stamp
        reason = self.problem
        if age is not None and not 0 <= age <= self.maximum_age:
            reason = "stale_observation"
        status = {"stamp":now,"frame_id":"warehouse","state":"ACTIVE" if reason is None else "WAITING",
                  "reason":reason,"observation_age":age,"processing_ms":self.processing_ms,
                  "accepted":self.accepted,"rejected":self.rejected,"superseded":self.superseded,
                  "occupied_cells":counts["occupied"],"free_cells":counts["free"],
                  "reset_sequence":self.reset_sequence}
        self.status_publisher.publish(String(json.dumps(status,allow_nan=False)))
        self.publish_plan(now, status, evidence, clock)

    def publish_plan(self, now, mapping_status, evidence=None, clock=None):
        # Same in-memory map as depth processing; no cloud reconstruction.
        clock = (lambda: now) if clock is None else clock
        header = Header(stamp=rospy.Time.from_sec(now), frame_id="warehouse")
        route = Path(header=header)
        state, reason, expanded = "HOLD", "envelope_not_configured", 0
        plan_started = time.perf_counter()
        if self.planner is not None:
            if self.package_attached and self.carried_envelope is None:
                reason = "carried_envelope_not_configured"
            elif self.local_goal is None:
                reason = "no_local_goal"
            elif self.latest_position is None:
                reason = "no_current_pose"
            elif mapping_status["state"] != "ACTIVE":
                reason = "map_" + str(mapping_status["reason"] or "unavailable")
            else:
                if self.goal_selector:
                    result = self.goal_selector.plan(self.latest_position, self.local_goal, now, evidence)
                else:
                    result = self.planner.plan(self.latest_position, self.local_goal, now, evidence=evidence)
                state, reason, expanded = result.state, result.reason, result.expanded
                if state == "READY":
                    for waypoint in result.path:
                        pose = PoseStamped(header=header)
                        pose.pose.position = Point(*waypoint)
                        pose.pose.orientation.w = 1.0
                        route.poses.append(pose)
        self.timings["planning_ms"] = (time.perf_counter()-plan_started)*1000
        details = {"stamp":now, "frame_id":"warehouse", "state":state,
                   "reason":reason, "waypoints":len(route.poses),
                   "expanded":expanded, "reset_sequence":self.reset_sequence,
                   "observation_stamp":self.last_stamp, "goal":self.local_goal}
        if self.goal_selector:
            details['request_goal'] = self.local_goal
            details['goal'] = self.goal_selector.goal
        details["package_attached"] = self.package_attached
        if self.planner is not None and self.latest_position is not None:
            diagnostic_started = time.perf_counter()
            camera = self.latest_camera or (None, None)
            details["start_clearance"] = self.planner.clearance_diagnostics(
                self.latest_position, now, *camera, evidence=evidence)
            self.timings["diagnostics_ms"] = (time.perf_counter()-diagnostic_started)*1000
        details["timing_ms"] = dict(self.timings)
        published_at = clock()
        details["timing_ms"]["plan_age_at_publish_ms"] = (published_at-now)*1000
        if self.last_stamp is not None:
            observation_age = published_at-self.last_stamp
            details["timing_ms"]["observation_age_at_publish_ms"] = observation_age*1000
            if state == "READY" and not 0 <= observation_age <= self.maximum_age:
                # A slow plan cannot turn an expired acquisition into approval.
                route.poses.clear()
                details.update(state="HOLD", reason="map_stale_observation", waypoints=0)
        message = String(json.dumps(details,allow_nan=False))
        # Build diagnostics/JSON first: no expensive work separates the pair.
        self.plan_path_publisher.publish(route)
        self.plan_status_publisher.publish(message)

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
