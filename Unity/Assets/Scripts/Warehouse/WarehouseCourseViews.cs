using UnityEngine;

public sealed class WarehouseCourseViews : MonoBehaviour
{
    public Transform drone;
    public Vector3 overviewPosition, overviewTarget, yardPosition, yardTarget;
    public Renderer[] roofs;
    int view = 2;
    void LateUpdate()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) view = 0;
        if (Input.GetKeyDown(KeyCode.Alpha2)) view = 1;
        if (Input.GetKeyDown(KeyCode.Alpha3)) view = 2;
        foreach (var roof in roofs) if (roof != null) roof.enabled = view == 1;
        Vector3 position = view == 0 ? overviewPosition : view == 2 ? yardPosition : drone.position + new Vector3(8, 5, -9);
        Vector3 target = view == 0 ? overviewTarget : view == 2 ? yardTarget : drone.position;
        transform.SetPositionAndRotation(position, Quaternion.LookRotation(target - position));
    }
}
