public static class DroneRosTopics
{
    public const string CurrentPosition = "/drone/current_position";
    public const string CommandPosition = "/drone/command_position";
    public const string ObstacleDistances = "/drone/obstacle_distances";
    public const string TravelHeading = "/drone/travel_heading";
    public const string PackageAction = "/drone/package_action";
    public const string PackageActionStatus = "/drone/package_action_status";
    public const string RgbImage = "/drone/camera/rgb/image_raw";
    public const string DepthImage = "/drone/camera/depth/image_raw";
    public const string CameraInfo = "/drone/camera/camera_info";
    public const string CameraPose = "/drone/camera/pose";
    public const string CurrentPose = "/drone/current_pose";
    public const string Transforms = "/tf";
}
