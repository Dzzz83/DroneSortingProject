"""ROS topic names shared by the drone sorting nodes."""

CURRENT_POSITION = "/drone/current_position"
TARGET_POSITION = "/drone/target_position"
COMMAND_POSITION = "/drone/command_position"
COMMAND_SPEED = "/drone/command_speed"
PICKUP_TRIAL_STATUS = "/drone/pickup_trial_status"

OBSTACLE_DISTANCES = "/drone/obstacle_distances"
TRAVEL_HEADING = "/drone/travel_heading"

PACKAGE_ACTION = "/drone/package_action"
PACKAGE_ACTION_STATUS = "/drone/package_action_status"

MISSION_STATE = "/drone/mission_state"

RGB_IMAGE = "/drone/camera/rgb/image_raw"
DEPTH_IMAGE = "/drone/camera/depth/image_raw"
CAMERA_INFO = "/drone/camera/camera_info"
CAMERA_POSE = "/drone/camera/pose"
CURRENT_POSE = "/drone/current_pose"
TRANSFORMS = "/tf"
PACKAGE_CANDIDATES = "/drone/package_candidates"

FORWARD_RGB_IMAGE = "/drone/forward_camera/rgb/image_raw"
FORWARD_DEPTH_IMAGE = "/drone/forward_camera/depth/image_raw"
FORWARD_CAMERA_INFO = "/drone/forward_camera/camera_info"
FORWARD_CAMERA_POSE = "/drone/forward_camera/pose"
FORWARD_DRONE_POSE = "/drone/forward_camera/drone_pose"
FORWARD_CAMERA_ENABLED = "/drone/avoidance/camera_enabled"
FORWARD_CAMERA_HEADING = "/drone/avoidance/camera_heading"

FORWARD_RANGE_MAX = "/drone/forward_camera/range_max"

# ROS-only map outputs; Unity does not consume these topics.
LOCAL_MAP_OCCUPIED = "/drone/local_map/occupied"
LOCAL_MAP_FREE = "/drone/local_map/free"
LOCAL_MAP_STATUS = "/drone/local_map/status"

# Proposed routes only; no control topic is published by the planner.
LOCAL_PLAN_GOAL = "/drone/local_planner/goal"
LOCAL_PLAN_PATH = "/drone/local_planner/path"
LOCAL_PLAN_STATUS = "/drone/local_planner/status"
LOCAL_PLAN_SCAN_HEADING = "/drone/local_planner/scan_heading"
COURSE_OBSERVATIONS = "/drone/course/observations"

COURSE_RESET = "/drone/course/reset"
# ROS-only navigation coordinator status; Unity does not consume it.
NAVIGATION_STATUS = "/drone/navigation/status"
