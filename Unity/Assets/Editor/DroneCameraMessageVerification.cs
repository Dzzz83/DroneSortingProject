using System;
using System.IO;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public static class DroneCameraMessageVerification
{
    [MenuItem("Drone Delivery/Verify Camera ROS Messages")]
    public static void Verify()
    {
        var drone = new GameObject("TemporaryCameraMessageProbe") { hideFlags = HideFlags.HideAndDontSave };
        try
        {
            DroneRgbCamera sensor = DroneRgbCamera.EnsureOnDrone(drone);
            drone.transform.position = new Vector3(34f, 6f, 25f);
            foreach (Quaternion rotation in new[] { Quaternion.identity, Quaternion.Euler(20f, 70f, -15f) })
            {
                drone.transform.rotation = rotation;
                var frame = new RgbDepthFrame(2, 2, new byte[12], new[] { 1f, 2f, float.NaN, 4f },
                    0, sensor.transform.position, sensor.transform.rotation);
                var stamp = new TimeMsg(123, 456);
                DroneCameraMessages.Bundle messages = DroneCameraMessages.Build(frame, sensor.SensorCamera,
                    drone.transform.position, drone.transform.rotation, 7, stamp);
                Require(messages.Rgb.step == 6 && messages.Depth.step == 8, "Incorrect image strides.");
                Require(messages.Rgb.header.stamp == messages.CameraPose.header.stamp &&
                    messages.Depth.header.seq == 7 && messages.Calibration.header.frame_id == messages.Rgb.header.frame_id,
                    "Observation metadata is not synchronized.");
                Require(messages.Calibration.K[0] > 0 && messages.Calibration.K[4] > 0 &&
                    messages.Calibration.K[2] == 0.5 && messages.Calibration.K[5] == 0.5,
                    "Invalid camera intrinsics.");

                Quaternion optical = QuaternionFrom(messages.CameraPose.pose.orientation);
                Vector3 localProbe = new Vector3(0.3f, -0.2f, 2f);
                Vector3 expected = DroneCameraMessages.ToRosWorld(
                    sensor.transform.TransformPoint(new Vector3(localProbe.x, -localProbe.y, localProbe.z)));
                Vector3 actual = PositionFrom(messages.CameraPose.pose.position) + optical * localProbe;
                Require(Vector3.Distance(expected, actual) < 0.0001f, "Optical/world handedness conversion failed.");

                var body = messages.Transforms.transforms[0].transform;
                var mount = messages.Transforms.transforms[1].transform;
                Quaternion bodyRotation = QuaternionFrom(body.rotation);
                Vector3 composedPosition = VectorFrom(body.translation) + bodyRotation * VectorFrom(mount.translation);
                Quaternion composedRotation = bodyRotation * QuaternionFrom(mount.rotation);
                Require(Vector3.Distance(composedPosition, PositionFrom(messages.CameraPose.pose.position)) < 0.0001f &&
                    Quaternion.Angle(composedRotation, optical) < 0.01f, "TF chain does not match the camera pose.");
                Require(Vector3.Distance(bodyRotation * Vector3.right,
                    DroneCameraMessages.ToRosWorld(drone.transform.TransformDirection(Vector3.back))) < 0.0001f,
                    "Drone body X does not point forward.");

                var serializer = new MessageSerializer();
                serializer.SerializeMessage(messages.Depth);
                ImageMsg roundTrip = new MessageDeserializer().DeserializeMessage<ImageMsg>(serializer.GetBytes());
                Require(roundTrip.data.Length == 16 && float.IsNaN(BitConverter.ToSingle(roundTrip.data, 8)),
                    "Depth serialization did not preserve NaN.");

                string directory = Path.Combine(Path.GetTempPath(), "drone-camera-message-verification");
                Directory.CreateDirectory(directory);
                SaveMessage(directory, "rgb.bin", messages.Rgb);
                SaveMessage(directory, "depth.bin", messages.Depth);
                SaveMessage(directory, "camera-info.bin", messages.Calibration);
                SaveMessage(directory, "camera-pose.bin", messages.CameraPose);
                SaveMessage(directory, "drone-pose.bin", messages.DronePose);
                SaveMessage(directory, "tf.bin", messages.Transforms);
                File.WriteAllText(Path.Combine(directory, "report.txt"),
                    "PASS: metadata, calibration, depth serialization, optical handedness, rotated drone, and TF composition.\n" +
                    "ROS1 binary fixtures are available for independent Python deserialization.\n");
            }
            Debug.Log("DRONE_CAMERA_MESSAGES_VERIFIED");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(drone);
        }
    }

    private static void SaveMessage(string directory, string filename, Message message)
    {
        var serializer = new MessageSerializer();
        serializer.SerializeMessage(message);
        File.WriteAllBytes(Path.Combine(directory, filename), serializer.GetBytes());
    }

    private static Quaternion QuaternionFrom(QuaternionMsg value) =>
        new Quaternion((float)value.x, (float)value.y, (float)value.z, (float)value.w);

    private static Vector3 PositionFrom(PointMsg value) =>
        new Vector3((float)value.x, (float)value.y, (float)value.z);

    private static Vector3 VectorFrom(Vector3Msg value) =>
        new Vector3((float)value.x, (float)value.y, (float)value.z);

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
