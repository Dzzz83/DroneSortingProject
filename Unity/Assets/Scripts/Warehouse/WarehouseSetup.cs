using System;
using DroneDelivery;
using Unity.Cinemachine;
using UnityEngine;

public static class WarehouseSetup
{
    public static void ConfigureDrone(GameObject drone)
    {
        if (drone == null)
        {
            throw new ArgumentNullException(nameof(drone));
        }

        if (drone.GetComponent<PackageHandling>() == null)
        {
            throw new InvalidOperationException(
                "ROS warehouse drone requires PackageHandling."
            );
        }

        EnsureComponent<DroneCommandSubscriber>(drone);
        EnsureComponent<DroneStatePublisher>(drone);
        EnsureComponent<DroneObstacleSensorPublisher>(drone);
        EnsureComponent<PackageActionBridge>(drone);
        DroneRgbCamera.EnsureOnDrone(drone);
        EnsureComponent<DroneCameraPublisher>(drone);

        Rigidbody body = drone.GetComponent<Rigidbody>();

        if (body != null)
        {
            body.isKinematic = true;
            body.useGravity = false;
        }
    }

    public static void ConfigureSpectatorCamera(Transform drone)
    {
        foreach (CinemachineCamera camera in
                 UnityEngine.Object.FindObjectsByType<CinemachineCamera>(FindObjectsSortMode.None))
        {
            if (camera.Follow != drone)
                continue;

            var spectator = camera.GetComponent<SpectatorCamera>();
            if (spectator == null)
                throw new InvalidOperationException("Warehouse camera requires a saved SpectatorCamera component.");
            spectator.Configure(drone);
            CinemachineDeoccluder deoccluder = camera.GetComponent<CinemachineDeoccluder>();
            if (deoccluder == null)
                deoccluder = camera.gameObject.AddComponent<CinemachineDeoccluder>();
            deoccluder.CollideAgainst = Physics.DefaultRaycastLayers;
            // Ignore the tracked drone's own body and gripper near its root.
            deoccluder.MinimumDistanceFromTarget = 1.25f;
            var avoidance = deoccluder.AvoidObstacles;
            avoidance.Enabled = true;
            avoidance.CameraRadius = 0.2f;
            avoidance.MinimumOcclusionTime = 0f;
            avoidance.DampingWhenOccluded = 0f;
            deoccluder.AvoidObstacles = avoidance;
            Debug.Log("WAREHOUSE_SPECTATOR_READY wide aisle view with camera deocclusion");
        }
    }

    private static void EnsureComponent<T>(GameObject drone)
        where T : Component
    {
        if (drone.GetComponent<T>() == null)
        {
            drone.AddComponent<T>();
        }
    }
}
