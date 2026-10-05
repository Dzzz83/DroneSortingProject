using UnityEditor.SceneManagement;
using UnityEngine;

public static class DeliveryDemoSceneLauncher
{
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
