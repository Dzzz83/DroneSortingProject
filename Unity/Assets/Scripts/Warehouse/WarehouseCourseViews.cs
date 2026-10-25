using System;
using UnityEngine;

[RequireComponent(typeof(Camera))]
public sealed class WarehouseCourseViews : MonoBehaviour
{
    public const string RoofLayerName = "CourseRoof";
    public Transform drone;
    public Vector3 overviewPosition, overviewTarget, yardPosition, yardTarget;
    public Renderer[] roofs;
    int view = 2;
    Camera spectator;
    int originalCullingMask;

    void Awake()
    {
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
        view = index;
        ApplyView();
    }

    void LateUpdate()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) view = 0;
        if (Input.GetKeyDown(KeyCode.Alpha2)) view = 1;
        if (Input.GetKeyDown(KeyCode.Alpha3)) view = 2;
        ApplyView();
    }

    void ApplyView()
    {
        if (spectator == null) return;
        // Only this camera cuts away the roof. Sensor rendering and physics keep it.
        int roofMask = 1 << LayerMask.NameToLayer(RoofLayerName);
        spectator.cullingMask = view == 1 ? originalCullingMask : originalCullingMask & ~roofMask;
        Vector3 position = view == 0 ? overviewPosition : view == 2 ? yardPosition : drone.position + new Vector3(8, 5, -9);
        Vector3 target = view == 0 ? overviewTarget : view == 2 ? yardTarget : drone.position;
        transform.SetPositionAndRotation(position, Quaternion.LookRotation(target - position));
    }

    void OnDisable()
    {
        if (spectator != null) spectator.cullingMask = originalCullingMask;
    }
}
