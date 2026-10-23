using System;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class DemoLauncher
{
    private static double idleSince;
    private static double nextStartupReport;

    // Used by the Linux launcher after the initial synchronous asset import.
    // Opening a scene alone does not mean background indexing/shaders are done.
    public static void Prepare()
    {
        OpenMainWarehouse();
        // Fresh editor layouts can lack a Game tab. Make the simulation view
        // available without resetting the user's other windows.
        if (!EditorApplication.ExecuteMenuItem("Window/General/Game"))
            Debug.LogWarning("Open Window > General > Game before pressing Play.");
        idleSince = EditorApplication.timeSinceStartup;
        nextStartupReport = idleSince + 30;
        EditorApplication.update -= WaitForEditorIdle;
        EditorApplication.update += WaitForEditorIdle;
    }

    private static void WaitForEditorIdle()
    {
        if (EditorApplication.isCompiling || EditorApplication.isUpdating ||
            ShaderUtil.anythingCompiling || Progress.running)
        {
            idleSince = EditorApplication.timeSinceStartup;
            if (idleSince >= nextStartupReport)
            {
                nextStartupReport = idleSince + 30;
                Debug.Log($"Camera demo waiting: importing={EditorApplication.isUpdating}, " +
                    $"compiling={EditorApplication.isCompiling}, shaders={ShaderUtil.anythingCompiling}.");
                foreach (var task in Progress.EnumerateItems())
                    if (task.running) Debug.Log("Camera demo background task: " + task.name);
            }
            return;
        }

        // Give delayed startup callbacks time to schedule their background work.
        if (EditorApplication.timeSinceStartup - idleSince < 10) return;
        EditorApplication.update -= WaitForEditorIdle;
        foreach (string guid in AssetDatabase.FindAssets("t:Shader", new[] { "Assets" }))
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            Shader shader = AssetDatabase.LoadAssetAtPath<Shader>(path);
            if (shader != null && ShaderUtil.ShaderHasError(shader))
                throw new InvalidOperationException("Warehouse shader failed to compile: " + path);
        }
        Debug.Log("CAMERA_DEMO_EDITOR_IDLE: imports, compilation and progress tasks settled.");
    }

    private const string WarehouseScenePath =
        "Assets/Scenes/MainWarehouse.unity";

    public static void OpenMainWarehouse()
    {
        OpenScene(WarehouseScenePath);
        foreach (GameObject root in UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects())
        {
            foreach (Transform child in root.GetComponentsInChildren<Transform>(true))
            {
                if (GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(child.gameObject) > 0)
                    throw new InvalidOperationException("Missing warehouse script on " + child.name);
            }
        }
        PackageHandling handling = UnityEngine.Object.FindAnyObjectByType<PackageHandling>();
        if (handling == null || handling.packageMount == null ||
            handling.pickupDetector == null || handling.gripper == null)
            throw new InvalidOperationException("Warehouse drone package references are incomplete.");
        if (Resources.Load<Shader>("DroneMetricDepth") == null)
            throw new InvalidOperationException("Warehouse metric-depth shader is missing.");
        Debug.Log("WAREHOUSE_SCENE_VERIFIED: scripts, package references and depth shader are present.");
    }

    private static void OpenScene(string scenePath)
    {
        EditorSceneManager.OpenScene(
            scenePath,
            OpenSceneMode.Single
        );

        Debug.Log(
            "Delivery demo scene ready: " + scenePath
        );
    }
}
