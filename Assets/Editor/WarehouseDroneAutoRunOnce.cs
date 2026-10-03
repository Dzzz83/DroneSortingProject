using System;
using System.Globalization;
using System.IO;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[InitializeOnLoad]
public static class WarehouseDroneAutoRunOnce
{
    private const string FlagPath = "Assets/DroneDelivery/RunWarehouseIntegration.flag";
    private const string DonePath = "Assets/DroneDelivery/WarehouseAutoRunResult.txt";
    private const string ScenePath = "Assets/Scenes/MainWarehouse.unity";
    private const string RootName = "Member2_DroneSystem";
    private const string DroneName = "DeliveryDrone_ATLAS01";
    private const string PackageName = "DeliveryPackage_Pickup";
    private const string DeliveryTargetName = "SortingZoneA";
    private const float DemoSpeed = 4f;
    private const bool AutoPlayAfterSetup = true;

    static WarehouseDroneAutoRunOnce()
    {
        EditorApplication.delayCall += TryRun;
    }

    private static void TryRun()
    {
        if (!File.Exists(ToAbsolutePath(FlagPath)))
            return;

        string status = "FAILED";
        string details = string.Empty;

        try
        {
            DeleteAssetFile(FlagPath);

            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            if (!scene.IsValid())
                throw new InvalidOperationException("Could not open " + ScenePath);

            if (GameObject.Find(RootName) == null)
            {
                WarehouseDroneIntegrator.Run();
                scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            }

            GameObject drone = RequireObject(DroneName);
            GameObject package = RequireObject(PackageName);
            GameObject root = RequireObject(RootName);
            GameObject deliveryTarget = RequireObject(DeliveryTargetName);

            PackageHandling handling = drone.GetComponent<PackageHandling>();
            DeliveryPackage deliveryPackage = package.GetComponent<DeliveryPackage>();
            if (handling == null || deliveryPackage == null)
                throw new InvalidOperationException("Drone or package is missing its delivery runtime component.");

            DeliveryDemo demo = root.GetComponent<DeliveryDemo>();
            if (demo == null)
                demo = root.AddComponent<DeliveryDemo>();

            demo.drone = handling;
            demo.package = deliveryPackage;
            demo.delivery = new Vector3(
                deliveryTarget.transform.position.x,
                package.transform.position.y,
                deliveryTarget.transform.position.z);
            demo.speed = DemoSpeed;
            demo.enabled = true;

            Selection.activeGameObject = drone;
            EditorGUIUtility.PingObject(drone);
            SceneView sceneView = SceneView.lastActiveSceneView;
            if (sceneView != null)
            {
                sceneView.FrameSelected();
                sceneView.Repaint();
            }

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            AssetDatabase.SaveAssets();

            status = "OK";
            details =
                "Scene: " + ScenePath + Environment.NewLine +
                "Root: " + root.name + Environment.NewLine +
                "Drone: " + Format(drone.transform.position) + Environment.NewLine +
                "Package: " + Format(package.transform.position) + Environment.NewLine +
                "Delivery target: " + DeliveryTargetName + " " + Format(demo.delivery) + Environment.NewLine +
                "Automatic mission: enabled" + Environment.NewLine;

            Debug.Log("WAREHOUSE_DRONE_AUTORUN_COMPLETE " + details.Replace(Environment.NewLine, " "));
            if (AutoPlayAfterSetup)
                EditorApplication.delayCall += EnterPlayMode;
        }
        catch (Exception exception)
        {
            details = exception.ToString();
            Debug.LogException(exception);
        }
        finally
        {
            string report =
                "Warehouse drone auto-run result" + Environment.NewLine +
                "Status: " + status + Environment.NewLine +
                "Time: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture) + Environment.NewLine +
                details;

            File.WriteAllText(ToAbsolutePath(DonePath), report);
            AssetDatabase.ImportAsset(DonePath);
            AssetDatabase.Refresh();
        }
    }

    private static GameObject RequireObject(string name)
    {
        GameObject value = GameObject.Find(name);
        if (value == null)
            throw new InvalidOperationException("Required object not found: " + name);
        return value;
    }

    private static void EnterPlayMode()
    {
        if (!EditorApplication.isPlayingOrWillChangePlaymode)
            EditorApplication.isPlaying = true;
    }

    private static string ToAbsolutePath(string assetPath)
    {
        string relativePath = assetPath.Replace('/', Path.DirectorySeparatorChar);
        return Path.Combine(Directory.GetCurrentDirectory(), relativePath);
    }

    private static void DeleteAssetFile(string assetPath)
    {
        string absolutePath = ToAbsolutePath(assetPath);
        if (File.Exists(absolutePath))
            File.Delete(absolutePath);

        string metaPath = absolutePath + ".meta";
        if (File.Exists(metaPath))
            File.Delete(metaPath);
    }

    private static string Format(Vector3 value)
    {
        return string.Format(
            CultureInfo.InvariantCulture,
            "({0:0.###}, {1:0.###}, {2:0.###})",
            value.x,
            value.y,
            value.z);
    }
}
