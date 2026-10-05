using System;
using DroneDelivery;
using UnityEngine;

public static class WarehouseRosSetup
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

        Rigidbody body = drone.GetComponent<Rigidbody>();

        if (body != null)
        {
            body.isKinematic = true;
            body.useGravity = false;
        }
    }

    public static void DisableDeliveryDemos()
    {
        DeliveryDemo[] demos =
            UnityEngine.Object.FindObjectsByType<DeliveryDemo>(
                FindObjectsSortMode.None
            );

        foreach (DeliveryDemo demo in demos)
        {
            demo.enabled = false;
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
