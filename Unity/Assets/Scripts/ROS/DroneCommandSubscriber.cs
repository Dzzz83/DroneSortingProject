using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneCommandSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PointMsg>(
            DroneRosTopics.CommandPosition,
            ReceivePosition
        );
    }

    private void ReceivePosition(PointMsg message)
    {
        if (!enabled) return;
        // ROS:
        // x = horizontal X
        // y = horizontal Y
        // z = vertical
        //
        // Unity:
        // x = horizontal X
        // y = vertical
        // z = horizontal Z

        Vector3 target = new Vector3(
            (float)message.x,
            (float)message.z,
            (float)message.y
        );
        if (!float.IsFinite(target.x) || !float.IsFinite(target.y) || !float.IsFinite(target.z))
        {
            Debug.LogWarning("Ignored non-finite ROS position command.");
            return;
        }
        var course = GetComponent<WarehouseCourse>();
        if (course != null) course.ObserveCommand(target);
        transform.position = target;
    }
}
