using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using UnityEngine;

/// <summary>Sensor message formatting; all package perception remains in ROS.</summary>
public static class DroneCameraMessages
{
    public const string WorldFrame = "warehouse";
    public const string BodyFrame = "drone_base";
    public const string OpticalFrame = "package_camera_optical";

    public sealed class Bundle
    {
        public ImageMsg Rgb;
        public ImageMsg Depth;
        public CameraInfoMsg Calibration;
        public PoseStampedMsg DronePose;
        public PoseStampedMsg CameraPose;
        public TFMessageMsg Transforms;
    }

    public static TimeMsg UtcStamp()
    {
        long ticks = DateTime.UtcNow.Ticks - new DateTime(1970, 1, 1).Ticks;
        return new TimeMsg((uint)(ticks / TimeSpan.TicksPerSecond),
            (uint)((ticks % TimeSpan.TicksPerSecond) * 100));
    }

    public static Bundle Build(RgbDepthFrame frame, Camera camera, Vector3 dronePosition,
        Quaternion droneRotation, uint sequence, TimeMsg stamp, string opticalFrame = OpticalFrame)
    {
        int pixelCount = checked(frame.Width * frame.Height);
        if (frame.Rgb.Length != pixelCount * 3 || frame.DepthMetres.Length != pixelCount)
            throw new ArgumentException("Frame dimensions do not match its pixel arrays.");

        byte[] depthBytes = new byte[pixelCount * sizeof(float)];
        Buffer.BlockCopy(frame.DepthMetres, 0, depthBytes, 0, depthBytes.Length);
        var opticalHeader = new HeaderMsg(sequence, stamp, opticalFrame);
        Vector3 bodyPosition = ToRosWorld(dronePosition);
        Vector3 cameraPosition = ToRosWorld(frame.CameraPosition);

        // Swapping world Y/Z reflects handedness. Convert basis vectors, not
        // quaternion components. Body axes: forward=-Unity Z, left=Unity X, up=Y.
        Quaternion bodyRotation = Quaternion.LookRotation(
            ToRosWorld(droneRotation * Vector3.up),
            ToRosWorld(droneRotation * Vector3.right));
        // Optical axes: right=Unity X, down=-Unity Y, forward=Unity Z.
        Quaternion opticalRotation = Quaternion.LookRotation(
            ToRosWorld(frame.CameraRotation * Vector3.forward),
            ToRosWorld(frame.CameraRotation * Vector3.down));
        Quaternion inverseBody = Quaternion.Inverse(bodyRotation);

        return new Bundle
        {
            Rgb = new ImageMsg(opticalHeader, (uint)frame.Height, (uint)frame.Width,
                "rgb8", 0, (uint)(frame.Width * 3), frame.Rgb),
            Depth = new ImageMsg(opticalHeader, (uint)frame.Height, (uint)frame.Width,
                "32FC1", (byte)(BitConverter.IsLittleEndian ? 0 : 1),
                (uint)(frame.Width * sizeof(float)), depthBytes),
            Calibration = Calibration(camera, frame.Width, frame.Height, opticalHeader),
            DronePose = Pose(bodyPosition, bodyRotation, sequence, stamp),
            CameraPose = Pose(cameraPosition, opticalRotation, sequence, stamp),
            Transforms = new TFMessageMsg(new[]
            {
                Transform(WorldFrame, BodyFrame, bodyPosition, bodyRotation, sequence, stamp),
                Transform(BodyFrame, opticalFrame, inverseBody * (cameraPosition - bodyPosition),
                    inverseBody * opticalRotation, sequence, stamp),
            }),
        };
    }

    public static Vector3 ToRosWorld(Vector3 unityPosition)
    {
        return new Vector3(unityPosition.x, unityPosition.z, unityPosition.y);
    }

    private static CameraInfoMsg Calibration(Camera camera, int width, int height, HeaderMsg header)
    {
        double fy = height / (2.0 * Math.Tan(camera.fieldOfView * Math.PI / 360.0));
        double fx = fy * width / (height * camera.aspect);
        // Pixel indices refer to pixel centres, not the image's outer edges.
        double cx = (width - 1) / 2.0;
        double cy = (height - 1) / 2.0;
        return new CameraInfoMsg
        {
            header = header, width = (uint)width, height = (uint)height,
            distortion_model = "plumb_bob", D = new double[5],
            K = new[] { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 },
            R = new[] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },
            P = new[] { fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0 },
        };
    }

    private static QuaternionMsg Rotation(Quaternion rotation)
    {
        return new QuaternionMsg(rotation.x, rotation.y, rotation.z, rotation.w);
    }

    private static PoseStampedMsg Pose(Vector3 position, Quaternion rotation, uint sequence, TimeMsg stamp)
    {
        return new PoseStampedMsg(new HeaderMsg(sequence, stamp, WorldFrame),
            new PoseMsg(new PointMsg(position.x, position.y, position.z), Rotation(rotation)));
    }

    private static TransformStampedMsg Transform(string parent, string child, Vector3 position,
        Quaternion rotation, uint sequence, TimeMsg stamp)
    {
        return new TransformStampedMsg(new HeaderMsg(sequence, stamp, parent), child,
            new TransformMsg(new Vector3Msg(position.x, position.y, position.z), Rotation(rotation)));
    }
}
