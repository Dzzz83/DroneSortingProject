using DroneDelivery;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class WarehouseBootstrap
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

        PackageHandling packageHandling =
            Object.FindFirstObjectByType<PackageHandling>();

        if (packageHandling == null)
        {
            Debug.LogError(
                "Warehouse ROS bootstrap: delivery drone not found."
            );
            return;
        }

        WarehouseSetup.ConfigureDrone(
            packageHandling.gameObject
        );
        WarehouseSetup.ConfigureSpectatorCamera(packageHandling.transform);

        Debug.Log(
            "WAREHOUSE_ROS_RUNTIME_READY " +
            "drone=" + packageHandling.gameObject.name
        );
    }
}
