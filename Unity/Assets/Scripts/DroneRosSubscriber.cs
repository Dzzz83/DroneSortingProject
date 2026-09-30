using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneRosSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PointMsg>(
            "/drone/command_position",
            ReceivePosition
        );
    }

    private void ReceivePosition(PointMsg message)
    {
        // ROS:
        // x = horizontal X
        // y = horizontal Y
        // z = vertical
        //
        // Unity:
        // x = horizontal X
        // y = vertical
        // z = horizontal Z

        transform.position = new Vector3(
            (float)message.x,
            (float)message.z,
            (float)message.y
        );
    }
}