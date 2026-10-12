using System;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

/// <summary>
/// Defines the warehouse RGB sensor's mount and lens. Image capture and ROS
/// publishing are separate responsibilities and are not enabled by this component.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Camera), typeof(HDAdditionalCameraData))]
public class DroneRgbCamera : MonoBehaviour
{
    public const string MountName = "PackageRgbCamera";

    // Unity drone-local axes: Y is up; the model's front is -Z.
    public static readonly Vector3 MountPosition = new Vector3(0f, -0.55f, -0.60f);
    public static readonly Quaternion MountRotation = Quaternion.Euler(55f, 0f, 0f);

    [SerializeField, Min(1)] private int imageWidth = 800;
    [SerializeField, Min(1)] private int imageHeight = 600;
    [SerializeField, Range(1f, 179f)] private float verticalFieldOfView = 110f;
    [SerializeField, Min(0.001f)] private float nearClipDistance = 0.05f;
    [SerializeField, Min(0.002f)] private float farClipDistance = 30f;

    public int ImageWidth => imageWidth;
    public int ImageHeight => imageHeight;
    public Camera SensorCamera => GetComponent<Camera>();

    public static DroneRgbCamera EnsureOnDrone(GameObject drone)
    {
        if (drone == null)
            throw new ArgumentNullException(nameof(drone));

        Transform mount = drone.transform.Find(MountName);
        if (mount == null)
        {
            mount = new GameObject(MountName).transform;
            mount.SetParent(drone.transform, false);
            mount.localPosition = MountPosition;
            mount.localRotation = MountRotation;
        }

        DroneRgbCamera sensor = mount.GetComponent<DroneRgbCamera>();
        if (sensor == null)
            sensor = mount.gameObject.AddComponent<DroneRgbCamera>();

        sensor.ApplyLensSettings();
        return sensor;
    }

    private void Awake()
    {
        ApplyLensSettings();
    }

    private void OnValidate()
    {
        imageWidth = Mathf.Max(1, imageWidth);
        imageHeight = Mathf.Max(1, imageHeight);
        nearClipDistance = Mathf.Max(0.001f, nearClipDistance);
        farClipDistance = Mathf.Max(nearClipDistance + 0.001f, farClipDistance);
        ApplyLensSettings();
    }

    public void ApplyLensSettings()
    {
        Camera camera = SensorCamera;
        if (camera == null)
            return;
        // Render on demand; never replace the spectator camera or render to screen.
        camera.enabled = false;
        camera.orthographic = false;
        camera.usePhysicalProperties = false;
        camera.fieldOfView = verticalFieldOfView;
        camera.aspect = (float)imageWidth / imageHeight;
        camera.nearClipPlane = nearClipDistance;
        camera.farClipPlane = farClipDistance;

        HDAdditionalCameraData hdCamera = GetComponent<HDAdditionalCameraData>();
        if (hdCamera != null)
            hdCamera.antialiasing = HDAdditionalCameraData.AntialiasingMode.None;
    }
}
