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
