using System;
using System.Collections.Generic;
using DroneDelivery;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Opt-in RGB-depth sensor with a ROS-controlled viewing direction.</summary>
[DisallowMultipleComponent]
public sealed class ForwardObstacleCamera : MonoBehaviour
{
    public const string OpticalFrame = "forward_camera_optical";
    public const string MountName = "ForwardObstacleCamera";
    public const float MinimumDepth = .2f;
    public const float MaximumDepth = 15f;
    [SerializeField, Range(1f, 10f)] float framesPerSecond = 6f;
    // The forward stream is opt-in and primarily consumed by the depth mapper.
    // Turn this off to restore the original rendered RGB+depth capture.
    [SerializeField] bool physicsDepthOnly = true;
    DroneRgbCamera sensor;
    RgbDepthCapture capture;
    ROSConnection ros;
    bool requested;
    Vector3 heading = Vector3.forward;
    double nextCapture;
    uint sequence;
    readonly Dictionary<MeshRenderer, MaterialPropertyBlock> originalBodyProperties =
        new Dictionary<MeshRenderer, MaterialPropertyBlock>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(DroneRosTopics.ForwardRgbImage, queue_size: 1);
        ros.RegisterPublisher<ImageMsg>(DroneRosTopics.ForwardDepthImage, queue_size: 1);
        ros.RegisterPublisher<Float32Msg>(DroneRosTopics.ForwardRangeMax, queue_size: 1);
        ros.RegisterPublisher<CameraInfoMsg>(DroneRosTopics.ForwardCameraInfo, queue_size: 1);
        ros.RegisterPublisher<PoseStampedMsg>(DroneRosTopics.ForwardCameraPose, queue_size: 1);
        ros.RegisterPublisher<PoseStampedMsg>(DroneRosTopics.ForwardDronePose, queue_size: 1);
        ros.Subscribe<BoolMsg>(DroneRosTopics.ForwardCameraEnabled, message => requested = message.data);
        ros.Subscribe<Vector3Msg>(DroneRosTopics.ForwardCameraHeading, ReceiveHeading);
    }

    void ReceiveHeading(Vector3Msg message)
    {
        var direction = new Vector3((float)message.x, (float)message.z, (float)message.y);
        if (float.IsFinite(direction.x) && float.IsFinite(direction.y) &&
            float.IsFinite(direction.z) && direction.sqrMagnitude > .000001f)
        {
            direction.Normalize();
            if ((direction - heading).sqrMagnitude > .000001f)
                nextCapture = 0; // A new view need not wait for the old view's cadence.
            heading = direction;
        }
    }

    void LateUpdate()
    {
        if (!requested || ros == null || !ros.HasConnectionThread || ros.HasConnectionError ||
            Time.unscaledTimeAsDouble < nextCapture) return;
        nextCapture = Time.unscaledTimeAsDouble + 1.0 / framesPerSecond;
        try
        {
            var bundle = CaptureObservation();
            ros.Publish(DroneRosTopics.ForwardRangeMax, new Float32Msg(MaximumDepth));
            ros.Publish(DroneRosTopics.ForwardCameraPose, bundle.CameraPose);
            ros.Publish(DroneRosTopics.ForwardDronePose, bundle.DronePose);
            ros.Publish(DroneRosTopics.ForwardCameraInfo, bundle.Calibration);
            if (bundle.Rgb != null) ros.Publish(DroneRosTopics.ForwardRgbImage, bundle.Rgb);
            ros.Publish(DroneRosTopics.ForwardDepthImage, bundle.Depth);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            requested = false;
            capture?.Dispose();
            capture = null;
        }
    }

    /// <summary>Capture one calibrated observation without requiring a ROS connection.</summary>
    public DroneCameraMessages.Bundle CaptureObservation()
    {
        if (sensor == null)
        {
            var mount = new GameObject(MountName);
            mount.transform.SetParent(transform, false);
            sensor = mount.AddComponent<DroneRgbCamera>();
            sensor.ConfigureLens(320, 240, 90f);
        }
        // Pan below the body, near the carried envelope's vertical centre.
        // This does not change the drone transform or spectator camera.
        // Looking straight up/down needs a non-parallel reference-up vector.
        var referenceUp = Mathf.Abs(Vector3.Dot(heading, Vector3.up)) > .99f
            ? Vector3.forward : Vector3.up;
        sensor.transform.SetPositionAndRotation(transform.position + Vector3.down * .5f,
            Quaternion.LookRotation(heading, referenceUp));
        RgbDepthFrame frame;
        if (physicsDepthOnly)
        {
            // No GPU render, depth shader, texture readback, or invented RGB.
            int width = sensor.ImageWidth, height = sensor.ImageHeight;
            frame = new RgbDepthFrame(width, height, Array.Empty<byte>(),
                new float[width * height], Time.timeAsDouble,
                sensor.SensorCamera.transform.position, sensor.SensorCamera.transform.rotation);
        }
        else
        {
            if (capture == null)
                capture = new RgbDepthCapture(sensor, MinimumDepth, MaximumDepth, reportClearRange: true);
            frame = CaptureWithoutOwnBody();
        }
        // Timestamp the actual physics acquisition, rather than retimestamping cached data.
        var stamp = DroneCameraMessages.UtcStamp();
        // Includes invisible solid collision proxies. Unsampled pixels remain NaN.
        PhysicsObstacleDepth.Measure(frame, sensor.SensorCamera, transform);
        return physicsDepthOnly
            ? DroneCameraMessages.BuildDepth(frame, sensor.SensorCamera,
                transform.position, transform.rotation, sequence++, stamp, OpticalFrame)
            : DroneCameraMessages.Build(frame, sensor.SensorCamera,
                transform.position, transform.rotation, sequence++, stamp, OpticalFrame);
    }

    RgbDepthFrame CaptureWithoutOwnBody()
    {
        var ownRenderers = GetComponentsInChildren<Renderer>();
        var original = new bool[ownRenderers.Length];
        try
        {
            for (int i = 0; i < ownRenderers.Length; i++)
            {
                original[i] = ownRenderers[i].forceRenderingOff;
                // An attached package is parented below the drone, but remains a
                // real sensor occluder. Hide only the drone's own geometry.
                if (ownRenderers[i].GetComponentInParent<DeliveryPackage>() == null)
                {
                    EnsureImmediateBodyMask(ownRenderers[i]);
                    ownRenderers[i].forceRenderingOff = true;
                }
            }
            return capture.Capture();
        }
        finally
        {
            // On-demand capture/readback finishes before returning. Other cameras
            // see the original renderer flags, even when the sensor capture fails.
            for (int i = 0; i < ownRenderers.Length; i++)
                if (ownRenderers[i] != null) ownRenderers[i].forceRenderingOff = original[i];
        }
    }

    void EnsureImmediateBodyMask(Renderer renderer)
    {
        if (!(renderer is MeshRenderer mesh) || originalBodyProperties.ContainsKey(mesh)) return;
        var original = new MaterialPropertyBlock();
        mesh.GetPropertyBlock(original);
        originalBodyProperties.Add(mesh, original);
        // HDRP's GPU Resident Drawer can retain body visibility across these
        // on-demand captures after movement/panning. A custom property block
        // uses the documented per-renderer CPU path, where the mask is immediate.
        // The marker is unused by materials; existing visual properties survive.
        var properties = new MaterialPropertyBlock();
        mesh.GetPropertyBlock(properties);
        properties.SetFloat("_SensorCaptureCpuRendering", 1f);
        mesh.SetPropertyBlock(properties);
    }

    void OnDisable()
    {
        capture?.Dispose(); capture = null;
        foreach (var entry in originalBodyProperties)
            if (entry.Key != null) entry.Key.SetPropertyBlock(entry.Value);
        originalBodyProperties.Clear();
    }
}
