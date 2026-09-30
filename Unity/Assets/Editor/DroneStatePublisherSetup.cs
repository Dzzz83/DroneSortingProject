using UnityEditor;
using UnityEngine;

public static class DroneStatePublisherSetup
{
    [MenuItem("Tools/Drone/Setup State Publisher")]
    public static void SetupStatePublisher()
    {
        DroneRosSubscriber subscriber =
            Object.FindFirstObjectByType<DroneRosSubscriber>();

        if (subscriber == null)
        {
            Debug.LogError(
                "Could not find a GameObject containing DroneRosSubscriber."
            );
            return;
        }

        GameObject drone = subscriber.gameObject;

        DroneStatePublisher publisher =
            drone.GetComponent<DroneStatePublisher>();

        if (publisher == null)
        {
            publisher = drone.AddComponent<DroneStatePublisher>();
            Debug.Log($"Added DroneStatePublisher to {drone.name}");
        }
        else
        {
            Debug.Log($"DroneStatePublisher already exists on {drone.name}");
        }

        EditorUtility.SetDirty(drone);
    }
}