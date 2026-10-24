using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Publishes synchronized sensor observations, without perception or navigation.</summary>
[DisallowMultipleComponent]
public class DroneCameraPublisher : MonoBehaviour
{
    [SerializeField, Range(0.1f, 10f)] private float framesPerSecond = 2f;
    [SerializeField, Min(0.001f)] private float minimumDepth = 0.2f;
    [SerializeField, Min(0.002f)] private float maximumDepth = 15f;

    private ROSConnection ros;
    private DroneRgbCamera sensor;
    private RgbDepthCapture capture;
    private double nextCaptureTime;
    private uint sequence;

    private void Start()
    {
        sensor = DroneRgbCamera.EnsureOnDrone(gameObject);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(DroneRosTopics.RgbImage, queue_size: 1);
        ros.RegisterPublisher<ImageMsg>(DroneRosTopics.DepthImage, queue_size: 1);
        ros.RegisterPublisher<CameraInfoMsg>(DroneRosTopics.CameraInfo, queue_size: 1);
        ros.RegisterPublisher<PoseStampedMsg>(DroneRosTopics.CameraPose, queue_size: 1);
        ros.RegisterPublisher<PoseStampedMsg>(DroneRosTopics.CurrentPose, queue_size: 1);
        ros.RegisterPublisher<TFMessageMsg>(DroneRosTopics.Transforms, queue_size: 1);
    }

    private void LateUpdate()
    {
        if (ros == null || !ros.HasConnectionThread || ros.HasConnectionError ||
            Time.unscaledTimeAsDouble < nextCaptureTime) return;

        // Never accumulate a backlog or burst to catch up after a slow capture.
        nextCaptureTime = Time.unscaledTimeAsDouble + 1.0 / framesPerSecond;
        try
        {
            if (capture == null) capture = new RgbDepthCapture(sensor, minimumDepth, maximumDepth);
            var stamp = DroneCameraMessages.UtcStamp();
            Vector3 dronePosition = transform.position;
            Quaternion droneRotation = transform.rotation;
            RgbDepthFrame frame = capture.Capture();
            DroneCameraMessages.Bundle messages = DroneCameraMessages.Build(frame,
                sensor.SensorCamera, dronePosition, droneRotation, sequence++, stamp);
            ros.Publish(DroneRosTopics.Transforms, messages.Transforms);
            ros.Publish(DroneRosTopics.CurrentPose, messages.DronePose);
            ros.Publish(DroneRosTopics.CameraPose, messages.CameraPose);
            ros.Publish(DroneRosTopics.CameraInfo, messages.Calibration);
            ros.Publish(DroneRosTopics.RgbImage, messages.Rgb);
            ros.Publish(DroneRosTopics.DepthImage, messages.Depth);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            // Fail visibly once instead of repeatedly stalling the main thread.
            enabled = false;
        }
    }

    private void OnDisable()
    {
        capture?.Dispose();
        capture = null;
    }

    private void OnValidate()
    {
        framesPerSecond = Mathf.Clamp(framesPerSecond, 0.1f, 10f);
        minimumDepth = Mathf.Max(0.001f, minimumDepth);
        maximumDepth = Mathf.Max(minimumDepth + 0.001f, maximumDepth);
    }
}
