using System;
using UnityEngine;

/// <summary>Course spectator views, free flight and an adjustable drone-follow offset.</summary>
[RequireComponent(typeof(Camera))]
public sealed class WarehouseCourseViews : MonoBehaviour
{
    public const string RoofLayerName = "CourseRoof";
    public Transform drone;
    public Vector3 overviewPosition, overviewTarget, yardPosition, yardTarget;
    public Renderer[] roofs = Array.Empty<Renderer>();
    [SerializeField, Min(.1f)] float moveSpeed = 6f;
    [SerializeField, Min(1f)] float fastMoveMultiplier = 3f;
    [SerializeField, Min(.1f)] float lookSensitivity = 2f;
    static readonly Vector3 DefaultFollowOffset = new Vector3(8,5,-9);
    Vector3 followOffset = DefaultFollowOffset;
    int view = 2;
    bool freeCamera, ownsCursor;
    float yaw, pitch;
    Camera spectator;
    int originalCullingMask;

    void Awake() => InitializeCamera();

    public void InitializeCamera()
    {
        if (spectator != null) return;
        spectator = GetComponent<Camera>();
        originalCullingMask = spectator.cullingMask;
        ConfigureRoofs();
    }

    public void ConfigureRoofs()
    {
        int roofLayer = LayerMask.NameToLayer(RoofLayerName);
        if (roofLayer < 0)
            throw new InvalidOperationException("Add the CourseRoof layer before opening the course.");
        foreach (var roof in roofs)
        {
            if (roof == null) continue;
            roof.enabled = true;
            roof.gameObject.layer = roofLayer;
        }
    }

    public void SelectView(int index)
    {
        if (index < 0 || index > 2) throw new ArgumentOutOfRangeException(nameof(index));
        ReleaseCursor();
        view = index;
        freeCamera = false;
        followOffset = DefaultFollowOffset;
        RefreshView();
    }

    void LateUpdate()
    {
        if (Application.isFocused)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1)) SelectView(0);
            if (Input.GetKeyDown(KeyCode.Alpha2) || Input.GetKeyDown(KeyCode.F)) SelectView(1);
            if (Input.GetKeyDown(KeyCode.Alpha3)) SelectView(2);
            if (Input.GetMouseButtonDown(1))
            {
                ownsCursor = true;
                yaw = transform.eulerAngles.y;
                pitch = Mathf.DeltaAngle(0,transform.eulerAngles.x);
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
            }
            if (ownsCursor && (!Input.GetMouseButton(1) || Input.GetKeyDown(KeyCode.Escape)))
                ReleaseCursor();
            if (ownsCursor)
            {
                var look = new Vector2(Input.GetAxis("Mouse X"),Input.GetAxis("Mouse Y"));
                var movement = new Vector3(Input.GetAxisRaw("Horizontal"),
                    (Input.GetKey(KeyCode.E) ? 1 : 0) - (Input.GetKey(KeyCode.Q) ? 1 : 0),
                    Input.GetAxisRaw("Vertical"));
                bool fast = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
                AdjustView(look,movement,fast,Time.unscaledDeltaTime);
            }
        }
        RefreshView();
    }

    public void AdjustView(Vector2 look, Vector3 movement, bool fast, float deltaTime)
    {
        float speed = moveSpeed * (fast ? fastMoveMultiplier : 1f);
        if (view == 1)
        {
            // Orbit the target; keep looking at it while retaining the custom
            // world-space offset as the drone continues through both warehouses.
            float distance = followOffset.magnitude;
            float azimuth = Mathf.Atan2(followOffset.x,followOffset.z) * Mathf.Rad2Deg;
            float elevation = Mathf.Asin(followOffset.y/distance) * Mathf.Rad2Deg;
            azimuth += look.x * lookSensitivity;
            elevation = Mathf.Clamp(elevation + look.y * lookSensitivity,-80,80);
            followOffset = Quaternion.Euler(-elevation,azimuth,0) * Vector3.forward * distance;
            Vector3 direction = transform.right*movement.x + Vector3.up*movement.y + transform.forward*movement.z;
            Vector3 adjusted = followOffset + Vector3.ClampMagnitude(direction,1) * speed * deltaTime;
            followOffset = adjusted.magnitude >= 1f ? adjusted : adjusted.normalized;
            if (followOffset.sqrMagnitude < .001f) followOffset = Vector3.back;
        }
        else
        {
            if (!freeCamera)
            {
                yaw = transform.eulerAngles.y;
                pitch = Mathf.DeltaAngle(0,transform.eulerAngles.x);
            }
            freeCamera = true;
            yaw += look.x * lookSensitivity;
            pitch = Mathf.Clamp(pitch-look.y*lookSensitivity,-85,85);
            transform.rotation = Quaternion.Euler(pitch,yaw,0);
            Vector3 direction = transform.right*movement.x + Vector3.up*movement.y + transform.forward*movement.z;
            transform.position += Vector3.ClampMagnitude(direction,1) * speed * deltaTime;
        }
    }

    public void RefreshView()
    {
        if (spectator == null || drone == null) return;
        // Only this camera cuts away the roof. Sensor rendering and physics keep it.
        int roofMask = 1 << LayerMask.NameToLayer(RoofLayerName);
        spectator.cullingMask = view == 1 ? originalCullingMask : originalCullingMask & ~roofMask;
        if (freeCamera) return;
        Vector3 position = view == 0 ? overviewPosition : view == 2 ? yardPosition : drone.position + followOffset;
        Vector3 target = view == 0 ? overviewTarget : view == 2 ? yardTarget : drone.position;
        transform.SetPositionAndRotation(position,Quaternion.LookRotation(target-position));
    }

    void ReleaseCursor()
    {
        if (!ownsCursor) return;
        ownsCursor = false;
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
    }

    void OnApplicationFocus(bool focused)
    {
        if (!focused) ReleaseCursor();
    }

    void OnDisable()
    {
        ReleaseCursor();
        if (spectator != null) spectator.cullingMask = originalCullingMask;
    }
}
