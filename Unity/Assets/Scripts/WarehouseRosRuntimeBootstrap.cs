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
        if (SceneManager.GetActiveScene().name != WarehouseSceneName)
        {
            return;
        }

        WarehouseRosSetup.DisableDeliveryDemos();

        PackageHandling packageHandling =
            Object.FindFirstObjectByType<PackageHandling>();

        if (packageHandling == null)
        {
            Debug.LogError(
                "Warehouse ROS bootstrap: delivery drone not found."
            );
            return;
        }

        WarehouseRosSetup.ConfigureDrone(
            packageHandling.gameObject
        );

        Debug.Log(
            "WAREHOUSE_ROS_RUNTIME_READY " +
            "drone=" + packageHandling.gameObject.name +
            " | DeliveryDemo disabled"
        );
    }
}
