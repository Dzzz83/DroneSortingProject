public static class DroneRosTopics
{
    public const string CurrentPosition = "/drone/current_position";
    public const string CommandPosition = "/drone/command_position";
    public const string ApprovedTrajectory = "/drone/approved_trajectory";
    public const string TrajectoryStatus = "/drone/trajectory_status";
    public const string CommandSpeed = "/drone/command_speed";
    public const string PickupTrialStatus = "/drone/pickup_trial_status";
    public const string ObstacleDistances = "/drone/obstacle_distances";
    public const string TravelHeading = "/drone/travel_heading";
    public const string PackageAction = "/drone/package_action";
    public const string PackageActionStatus = "/drone/package_action_status";
    public const string PackageCameraEnabled = "/drone/camera/enabled";
    public const string RgbImage = "/drone/camera/rgb/image_raw";
    public const string DepthImage = "/drone/camera/depth/image_raw";
    public const string CameraInfo = "/drone/camera/camera_info";
    public const string CameraPose = "/drone/camera/pose";
    public const string PackageCandidates = "/drone/package_candidates";
    public const string CurrentPose = "/drone/current_pose";
    public const string Transforms = "/tf";
    public const string ForwardRgbImage = "/drone/forward_camera/rgb/image_raw";
    public const string ForwardDepthImage = "/drone/forward_camera/depth/image_raw";
    public const string ForwardRangeMax = "/drone/forward_camera/range_max";
    public const string ForwardCameraInfo = "/drone/forward_camera/camera_info";
    public const string ForwardCameraPose = "/drone/forward_camera/pose";
    public const string ForwardDronePose = "/drone/forward_camera/drone_pose";
    public const string ForwardCameraEnabled = "/drone/avoidance/camera_enabled";
    public const string ForwardCameraHeading = "/drone/avoidance/camera_heading";
}
