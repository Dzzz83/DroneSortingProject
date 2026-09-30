
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

public static class WarehouseSceneBuilder
{
    [MenuItem("Tools/Drone/Generate Obstacle Course")]
    public static void Generate()
    {
        // Remove the previously generated course.
        GameObject oldCourse =
            GameObject.Find("ObstacleCourse");

        if (oldCourse != null)
            Undo.DestroyObjectImmediate(oldCourse);

        // Create a parent for all generated objects.
        GameObject course = new GameObject("ObstacleCourse");
        Undo.RegisterCreatedObjectUndo(
            course, "Generate Obstacle Course"
        );

        // Create floor.
        CreateCube(
            "WarehouseFloor",
            new Vector3(9, -0.25f, 0),
            new Vector3(20, 0.5f, 6),
            course.transform
        );

        // Create warehouse ceiling.
        GameObject ceiling = CreateCube(
            "WarehouseCeiling",
            new Vector3(9, 5.5f, 0),
            new Vector3(20, 0.5f, 6),
            course.transform
        );

        // Create obstacles.
        GameObject ground1 = CreateCube(
            "GroundObstacle1",
            new Vector3(4, 1.25f, 0),
            new Vector3(1, 2.5f, 2),
            course.transform
        );

        GameObject elevated = CreateCube(
            "ElevatedObstacle",
            new Vector3(9, 3.5f, 0),
            new Vector3(1, 3.5f, 2),
            course.transform
        );

        GameObject ground2 = CreateCube(
            "GroundObstacle2",
            new Vector3(14, 1.25f, 0),
            new Vector3(1, 2.5f, 2),
            course.transform
        );

        // Create and assign the Obstacle layer.
        int obstacleLayer = GetOrCreateLayer("Obstacle");

        ground1.layer = obstacleLayer;
        elevated.layer = obstacleLayer;
        ground2.layer = obstacleLayer;
        ceiling.layer = obstacleLayer;

        // Create start and destination markers.
        CreateMarker(
            "StartZone",
            new Vector3(0, 0.04f, 0),
            course.transform
        );

        CreateMarker(
            "DestinationZone",
            new Vector3(18, 0.04f, 0),
            course.transform
        );

        // Position the existing drone.
        GameObject drone = GameObject.Find("Drone");

        if (drone == null)
            drone = GameObject.Find("DroneCube");

        if (drone != null)
        {
            Undo.RecordObject(drone.transform, "Position Drone");

            drone.transform.position =
                new Vector3(0, 2, 0);

            drone.transform.localScale =
                new Vector3(0.5f, 0.5f, 0.5f);
        }
        else
        {
            Debug.LogWarning(
                "Drone not found. Rename your existing " +
                "drone GameObject to Drone."
            );
        }

        // Mark the scene as modified so it can be saved.
        EditorSceneManager.MarkSceneDirty(
            SceneManager.GetActiveScene()
        );

        Debug.Log("Obstacle course generated successfully!");
    }

    private static GameObject CreateCube(
        string objectName,
        Vector3 position,
        Vector3 scale,
        Transform parent)
    {
        GameObject obj =
            GameObject.CreatePrimitive(PrimitiveType.Cube);

        obj.name = objectName;
        obj.transform.SetParent(parent);
        obj.transform.position = position;
        obj.transform.localScale = scale;

        return obj;
    }

    private static void CreateMarker(
        string objectName,
        Vector3 position,
        Transform parent)
    {
        GameObject marker =
            GameObject.CreatePrimitive(PrimitiveType.Cylinder);

        marker.name = objectName;
        marker.transform.SetParent(parent);
        marker.transform.position = position;
        marker.transform.localScale =
            new Vector3(0.75f, 0.04f, 0.75f);

        Object.DestroyImmediate(
            marker.GetComponent<Collider>()
        );
    }

    private static int GetOrCreateLayer(string layerName)
    {
        int existingLayer =
            LayerMask.NameToLayer(layerName);

        if (existingLayer >= 0)
            return existingLayer;

        Object[] assets = AssetDatabase.LoadAllAssetsAtPath(
            "ProjectSettings/TagManager.asset"
        );

        SerializedObject tagManager =
            new SerializedObject(assets[0]);

        SerializedProperty layers =
            tagManager.FindProperty("layers");

        for (int i = 8; i < 32; i++)
        {
            SerializedProperty layer =
                layers.GetArrayElementAtIndex(i);

            if (string.IsNullOrEmpty(layer.stringValue))
            {
                layer.stringValue = layerName;
                tagManager.ApplyModifiedProperties();

                return i;
            }
        }

        throw new System.InvalidOperationException(
            "No free Unity layers are available."
        );
    }
}