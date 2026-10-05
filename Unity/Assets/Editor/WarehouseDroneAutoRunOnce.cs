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
            GameObject root = RequireObject(RootName);

            if (drone.GetComponent<PackageHandling>() == null)
                throw new InvalidOperationException("Drone is missing PackageHandling.");

            EnsureComponent<DroneCommandSubscriber>(drone);
            EnsureComponent<DroneStatePublisher>(drone);
            EnsureComponent<DroneObstacleSensorPublisher>(drone);
            EnsureComponent<PackageActionBridge>(drone);

            foreach (DeliveryDemo demo in root.GetComponents<DeliveryDemo>())
                UnityEngine.Object.DestroyImmediate(demo);

            var body = drone.GetComponent<Rigidbody>();
            if (body != null)
            {
                body.isKinematic = true;
                body.useGravity = false;
            }

            Selection.activeGameObject = drone;
            EditorGUIUtility.PingObject(drone);

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            AssetDatabase.SaveAssets();

            status = "OK";
            details =
                "Scene: " + ScenePath + Environment.NewLine +
                "Root: " + root.name + Environment.NewLine +
                "Drone: " + Format(drone.transform.position) + Environment.NewLine +
                "Control: ROS" + Environment.NewLine +
                "DeliveryDemo: disabled/removed" + Environment.NewLine +
                "Automatic Play Mode: disabled; start ROS first." + Environment.NewLine;

            Debug.Log(
                "WAREHOUSE_ROS_SETUP_COMPLETE " +
                details.Replace(Environment.NewLine, " "));
        }
        catch (Exception exception)
        {
            details = exception.ToString();
            Debug.LogException(exception);
        }
        finally
        {
            string report =
                "Warehouse ROS integration result" + Environment.NewLine +
                "Status: " + status + Environment.NewLine +
                "Time: " + DateTime.Now.ToString(
                    "yyyy-MM-dd HH:mm:ss",
                    CultureInfo.InvariantCulture) + Environment.NewLine +
                details;

            File.WriteAllText(ToAbsolutePath(DonePath), report);
            AssetDatabase.ImportAsset(DonePath);
            AssetDatabase.Refresh();
        }
    }

    private static void EnsureComponent<T>(GameObject target)
        where T : Component
    {
        if (target.GetComponent<T>() == null)
            target.AddComponent<T>();
    }

    private static GameObject RequireObject(string name)
    {
        GameObject value = GameObject.Find(name);
        if (value == null)
            throw new InvalidOperationException("Required object not found: " + name);
        return value;
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
