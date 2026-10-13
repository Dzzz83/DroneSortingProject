using UnityEngine;

/// <summary>Wide warehouse viewing camera; independent of the drone's sensor.</summary>
public sealed class WarehouseSpectatorCamera : MonoBehaviour
{
    private Transform drone;

    public static Vector3 PreferredPosition(Vector3 dronePosition)
    {
        // Stay in the aisle and below overhead steelwork. World offsets keep
        // the route in view even when the drone changes its heading.
        // Stop before the overhang at the far end of the warehouse. The
        // fixed aisle X keeps the camera out of the destination's box stacks.
        return new Vector3(34f, 6.5f, Mathf.Min(dronePosition.z + 8f, 50f));
    }

    public static Quaternion ViewRotation(Vector3 dronePosition, Vector3 cameraPosition)
    {
        // Frame the drone and ground beneath it from a wide aisle view.
        Vector3 focus = dronePosition + new Vector3(0f, -1.5f, 0f);
        return Quaternion.LookRotation(focus - cameraPosition, Vector3.up);
    }

    public void Configure(Transform target)
    {
        drone = target;
        Vector3 position = PreferredPosition(drone.position);
        transform.SetPositionAndRotation(position, ViewRotation(drone.position, position));
    }

    private void Update()
    {
        if (drone == null) return;
        Vector3 position = Vector3.Lerp(transform.position, PreferredPosition(drone.position),
            1f - Mathf.Exp(-5f * Time.deltaTime));
        transform.SetPositionAndRotation(position, ViewRotation(drone.position, position));
    }
}
