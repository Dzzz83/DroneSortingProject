using DroneDelivery;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class WarehouseRosRuntimeBootstrap
{
    private const string WarehouseSceneName = "MainWarehouse";

    [RuntimeInitializeOnLoadMethod(
        RuntimeInitializeLoadType.AfterSceneLoad
    )]
    private static void ConfigureWarehouse()
    {
        if (
            SceneManager.GetActiveScene().name
            != WarehouseSceneName
        )
        {
            return;
        }

        DeliveryDemo[] demos =
            Object.FindObjectsByType<DeliveryDemo>(
                FindObjectsSortMode.None
            );

        foreach (DeliveryDemo demo in demos)
        {
            demo.enabled = false;
        }

        PackageHandling packageHandling =
            Object.FindFirstObjectByType<PackageHandling>();

        if (packageHandling == null)
        {
            Debug.LogError(
                "Warehouse ROS bootstrap: " +
                "PackageHandling drone not found."
            );
            return;
        }

        GameObject drone =
            packageHandling.gameObject;

        EnsureComponent<DroneCommandSubscriber>(
            drone
        );
        EnsureComponent<DroneStatePublisher>(
            drone
        );
        EnsureComponent<DroneObstacleSensorPublisher>(
            drone
        );
        EnsureComponent<PackageActionBridge>(
            drone
        );

        Rigidbody body =
            drone.GetComponent<Rigidbody>();

        if (body != null)
        {
            body.isKinematic = true;
            body.useGravity = false;
        }

        Debug.Log(
            "WAREHOUSE_ROS_RUNTIME_READY " +
            "drone=" + drone.name +
            " | DeliveryDemo disabled"
        );
    }

    private static void EnsureComponent<T>(
        GameObject gameObject
    )
        where T : Component
    {
        if (gameObject.GetComponent<T>() == null)
        {
            gameObject.AddComponent<T>();
        }
    }
}
