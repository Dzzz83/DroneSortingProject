using System;
using DroneDelivery;
using UnityEditor;
using UnityEngine;

/// <summary>Read-only editor audit of the actual in-scene drone envelope.</summary>
public static class DepthEnvelopeAudit
{
    [MenuItem("Drone Delivery/Report Depth Planner Envelope")]
    public static void Report()
    {
        if (!EditorApplication.isPlaying)
        {
            Debug.LogWarning("Enter Play before measuring the runtime drone envelope.");
            return;
        }
        var drone = UnityEngine.Object.FindAnyObjectByType<PackageHandling>();
        if (drone == null) throw new InvalidOperationException("No drone PackageHandling found.");
        Vector3 root = drone.transform.position;
        bool found = false;
        Bounds bounds = new Bounds(root, Vector3.zero);
        int colliders = 0;
        int renderers = 0;
        foreach (var component in drone.GetComponentsInChildren<Collider>(true))
        {
            if (!component.enabled || !component.gameObject.activeInHierarchy) continue;
            if (!found) { bounds = component.bounds; found = true; }
            else bounds.Encapsulate(component.bounds);
            colliders++;
        }
        foreach (var component in drone.GetComponentsInChildren<Renderer>(true))
        {
            if (!component.enabled || !component.gameObject.activeInHierarchy) continue;
            if (!found) { bounds = component.bounds; found = true; }
            else bounds.Encapsulate(component.bounds);
            renderers++;
        }
        if (!found) throw new InvalidOperationException("No enabled runtime geometry found.");
        float halfX = Mathf.Max(Mathf.Abs(bounds.min.x - root.x), Mathf.Abs(bounds.max.x - root.x));
        float halfY = Mathf.Max(Mathf.Abs(bounds.min.z - root.z), Mathf.Abs(bounds.max.z - root.z));
        float below = Mathf.Max(0f, root.y - bounds.min.y);
        float above = Mathf.Max(0f, bounds.max.y - root.y);
        Debug.Log($"DEPTH_ENVELOPE_AUDIT counts={colliders} colliders, {renderers} renderers " +
            $"package_attached={drone.HasPackage} " +
            $"ROS raw [half_x, half_y, below, above] = [{halfX:F3}, {halfY:F3}, {below:F3}, {above:F3}] m. " +
            "This is a bounding measurement, NOT a certified flight-clearance setting. " +
            "Add a justified margin to each dimension before enabling depth motion.");
    }
}
