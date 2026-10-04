using UnityEngine;

public class ObstacleCourseCamera : MonoBehaviour
{
    [Header("Camera Settings")]
    public float horizontalPadding = 1.5f;
    public float verticalPadding = 1.25f;
    public float cameraDistance = 25f;

    private Camera sceneCamera;
    private Bounds courseBounds;
    private Quaternion initialRotation;

    // Automatically configure the camera when Play starts.
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Initialize()
    {
        GameObject course = GameObject.Find("ObstacleCourse");
        Camera mainCamera = Camera.main;

        if (course == null || mainCamera == null)
        {
            Debug.LogWarning("Camera setup: obstacle course or Main Camera not found.");
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

        // Cache the rotation angle configured in the Scene View / Inspector
        initialRotation = transform.rotation;

        GameObject course = GameObject.Find("ObstacleCourse");

        if (course == null)
        {
            enabled = false;
            return;
        }

        Renderer[] renderers = course.GetComponentsInChildren<Renderer>();

        if (renderers.Length == 0)
        {
            enabled = false;
            return;
        }

        // Calculate boundaries of the entire obstacle course
        courseBounds = renderers[0].bounds;

        foreach (Renderer renderer in renderers)
        {
            courseBounds.Encapsulate(renderer.bounds);
        }

        sceneCamera.nearClipPlane = 0.1f;
        sceneCamera.farClipPlane = 1000f;

        UpdateCamera();

        Debug.Log("Camera configured: entire obstacle course is visible.");
    }

    private void LateUpdate()
    {
        UpdateCamera();
    }

    private void UpdateCamera()
    {
        if (sceneCamera == null)
            return;

        // Apply the saved rotation angle set in the Scene view
        transform.rotation = initialRotation;

        // Position the camera relative to its current rotation vector
        Vector3 center = courseBounds.center;
        transform.position = center - (transform.forward * cameraDistance);

        // Adjust orthographic sizing if orthographic mode is used
        if (sceneCamera.orthographic)
        {
            float requiredHeight = courseBounds.extents.y + verticalPadding;
            float requiredWidth = (courseBounds.extents.x + horizontalPadding) / Mathf.Max(sceneCamera.aspect, 0.1f);
            sceneCamera.orthographicSize = Mathf.Max(requiredHeight, requiredWidth);
        }
    }
}