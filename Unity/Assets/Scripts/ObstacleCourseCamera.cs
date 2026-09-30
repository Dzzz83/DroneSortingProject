using UnityEngine;

public class ObstacleCourseCamera : MonoBehaviour
{
    [Header("Camera Settings")]
    public float horizontalPadding = 1.5f;
    public float verticalPadding = 1.25f;
    public float cameraDistance = 25f;

    private Camera sceneCamera;
    private Bounds courseBounds;

    // Automatically configure the camera when Play starts.
    // No manual component attachment required.
    [RuntimeInitializeOnLoadMethod(
        RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Initialize()
    {
        GameObject course =
            GameObject.Find("ObstacleCourse");

        Camera mainCamera = Camera.main;

        if (course == null || mainCamera == null)
        {
            Debug.LogWarning(
                "Camera setup: obstacle course or Main Camera not found."
            );

            return;
        }

        if (mainCamera.GetComponent<ObstacleCourseCamera>() == null)
        {
            mainCamera.gameObject.AddComponent<ObstacleCourseCamera>();
        }
    }

    private void Awake()
    {
        sceneCamera = GetComponent<Camera>();

        GameObject course =
            GameObject.Find("ObstacleCourse");

        if (course == null)
        {
            enabled = false;
            return;
        }

        Renderer[] renderers =
            course.GetComponentsInChildren<Renderer>();

        if (renderers.Length == 0)
        {
            enabled = false;
            return;
        }

        // Calculate the boundaries of the entire
        // obstacle course, including the floor,
        // ceiling, obstacles, and destination markers.
        courseBounds = renderers[0].bounds;

        foreach (Renderer renderer in renderers)
        {
            courseBounds.Encapsulate(renderer.bounds);
        }

        sceneCamera.orthographic = true;

        sceneCamera.nearClipPlane = 0.1f;
        sceneCamera.farClipPlane = 100f;

        UpdateCamera();

        Debug.Log(
            "Camera configured: entire obstacle course is visible."
        );
    }

    private void LateUpdate()
    {
        // Maintain the correct framing even when
        // the Game window is resized.
        UpdateCamera();
    }

    private void UpdateCamera()
    {
        if (sceneCamera == null)
            return;

        // Position the camera in front of the course.
        Vector3 center = courseBounds.center;

        transform.position = new Vector3(
            center.x,
            center.y,
            courseBounds.min.z - cameraDistance
        );

        // Look directly toward the obstacle course.
        transform.rotation = Quaternion.identity;

        // Calculate the camera size needed to fit
        // the entire course horizontally and vertically.
        float requiredHeight =
            courseBounds.extents.y + verticalPadding;

        float requiredWidth =
            (courseBounds.extents.x + horizontalPadding)
            / Mathf.Max(sceneCamera.aspect, 0.1f);

        sceneCamera.orthographicSize =
            Mathf.Max(requiredHeight, requiredWidth);
    }
}