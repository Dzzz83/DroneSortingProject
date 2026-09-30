using UnityEditor;
using UnityEngine;

public static class DroneObstacleSensorSetup
{
    [MenuItem("Tools/Drone/Setup Obstacle Sensors")]
    public static void SetupObstacleSensors()
    {
        DroneRosSubscriber subscriber =
            Object.FindFirstObjectByType<DroneRosSubscriber>();

        if (subscriber == null)
        {
            Debug.LogError(
                "Could not find the drone with DroneRosSubscriber."
            );
            return;
        }

        GameObject drone = subscriber.gameObject;

        DroneObstacleSensorPublisher sensor =
            drone.GetComponent<DroneObstacleSensorPublisher>();

        if (sensor == null)
        {
            drone.AddComponent<DroneObstacleSensorPublisher>();

            Debug.Log(
                $"Added obstacle sensors to {drone.name}"
            );
        }
        else
        {
            Debug.Log(
                $"Obstacle sensors already exist on {drone.name}"
            );
        }

        EditorUtility.SetDirty(drone);
    }
}