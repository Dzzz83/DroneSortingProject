using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class DeliveryDemoSceneLauncher
{
    private static double idleSince;
    private static double nextStartupReport;

    // Used by the Linux launcher after the initial synchronous asset import.
    // Opening a scene alone does not mean background indexing/shaders are done.
    public static void PrepareMainWarehouseForCameraDemo()
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
        Debug.Log("CAMERA_DEMO_EDITOR_IDLE: imports, compilation and progress tasks settled.");
    }

    private const string SampleScenePath =
        "Assets/Scenes/SampleScene.unity";

    private const string WarehouseScenePath =
        "Assets/Scenes/MainWarehouse.unity";

    public static void OpenSampleScene()
    {
        OpenScene(SampleScenePath);
    }

    public static void OpenMainWarehouse()
    {
        OpenScene(WarehouseScenePath);
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
