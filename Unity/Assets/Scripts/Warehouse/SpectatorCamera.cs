using Unity.Cinemachine;
using UnityEngine;

/// <summary>Drone-following warehouse view with independent free-flight controls.</summary>
public sealed class SpectatorCamera : MonoBehaviour
{
    [Header("Follow view (Unity world coordinates)")]
    [Tooltip("Fixed world X and Y for the aisle view. Unity Y is height.")]
    [SerializeField] private Vector2 aislePosition = new Vector2(34f, 6.5f);
    [SerializeField] private float followDistance = 8f;
    [SerializeField] private float maximumZ = 50f;
    [SerializeField] private Vector3 focusOffset = new Vector3(0f, -1.5f, 0f);
    [SerializeField, Min(0.1f)] private float followSmoothing = 5f;

    [Header("Free camera controls")]
    [SerializeField, Min(0.1f)] private float moveSpeed = 6f;
    [SerializeField, Min(1f)] private float fastMoveMultiplier = 3f;
    [SerializeField, Min(0.1f)] private float lookSensitivity = 2f;

    private Transform drone;
    private bool manualControl;
    private bool ownsCursor;
    private float yaw;
    private float pitch;

    public Vector3 PreferredPosition(Vector3 dronePosition)
    {
        // Keep the default view in the aisle, below steelwork and before the overhang.
        return new Vector3(aislePosition.x, aislePosition.y, Mathf.Min(dronePosition.z + followDistance, maximumZ));
    }

    public Quaternion ViewRotation(Vector3 dronePosition, Vector3 cameraPosition)
    {
        Vector3 focus = dronePosition + focusOffset;
        return Quaternion.LookRotation(focus - cameraPosition, Vector3.up);
    }

    public void Configure(Transform target)
    {
        drone = target;
        ResumeFollowing();
    }

    private void Update()
    {
        if (drone == null)
            return;

        if (Application.isFocused)
        {
            if (Input.GetKeyDown(KeyCode.F))
                ResumeFollowing();
            else if (Input.GetMouseButtonDown(1))
                BeginManualControl();

            if (ownsCursor && (!Input.GetMouseButton(1) || Input.GetKeyDown(KeyCode.Escape)))
                ReleaseCursor();

            if (manualControl && ownsCursor)
            {
                Vector2 look = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
                Vector3 movement = new Vector3(Input.GetAxisRaw("Horizontal"),
                    (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f),
                    Input.GetAxisRaw("Vertical"));
                bool fast = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
                MoveManually(look, movement, fast, Time.unscaledDeltaTime);
            }
        }

        // Releasing the mouse leaves the view where the user placed it.
        if (manualControl)
            return;

        Vector3 position = Vector3.Lerp(transform.position, PreferredPosition(drone.position),
            1f - Mathf.Exp(-followSmoothing * Time.unscaledDeltaTime));
        transform.SetPositionAndRotation(position, ViewRotation(drone.position, position));
    }

    private void BeginManualControl()
    {
        manualControl = true;
        yaw = transform.eulerAngles.y;
        pitch = Mathf.DeltaAngle(0f, transform.eulerAngles.x);
        SetDeocclusion(false);
        ownsCursor = true;
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    private void MoveManually(Vector2 look, Vector3 movement, bool fast, float deltaTime)
    {
        // Mouse axes already represent a frame's delta; only translation uses elapsed time.
        yaw += look.x * lookSensitivity;
        pitch = Mathf.Clamp(pitch - look.y * lookSensitivity, -85f, 85f);
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 direction = transform.right * movement.x + Vector3.up * movement.y +
            transform.forward * movement.z;
        float speed = moveSpeed * (fast ? fastMoveMultiplier : 1f);
        transform.position += Vector3.ClampMagnitude(direction, 1f) * speed * deltaTime;
    }

    private void ResumeFollowing()
    {
        manualControl = false;
        ReleaseCursor();
        SetDeocclusion(true);
        if (drone == null)
            return;
        Vector3 position = PreferredPosition(drone.position);
        transform.SetPositionAndRotation(position, ViewRotation(drone.position, position));
    }

    private void SetDeocclusion(bool enabled)
    {
        // Automatic obstacle correction would otherwise pull the manual view back to the drone.
        CinemachineDeoccluder deoccluder = GetComponent<CinemachineDeoccluder>();
        if (deoccluder != null)
            deoccluder.enabled = enabled;
    }

    private void ReleaseCursor()
    {
        if (!ownsCursor)
            return;
        ownsCursor = false;
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
    }

    private void OnApplicationFocus(bool focused)
    {
        if (!focused)
            ReleaseCursor();
    }

    private void OnDisable()
    {
        ReleaseCursor();
    }
}
