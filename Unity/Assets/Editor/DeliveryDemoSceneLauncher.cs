using UnityEditor.SceneManagement;
using UnityEngine;

public static class DeliveryDemoSceneLauncher
{
    private const string ScenePath =
        "Assets/Scenes/SampleScene.unity";

    public static void OpenSampleScene()
    {
        EditorSceneManager.OpenScene(
            ScenePath,
            OpenSceneMode.Single
        );

        Debug.Log(
            "Delivery demo scene ready: " + ScenePath
        );
    }
}