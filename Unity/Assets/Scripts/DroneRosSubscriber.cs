using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneRosSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private Vector3 targetPosition;

    [SerializeField]
    private float moveSpeed = 2.0f;

    private void Start()
    {
        // get or create connection
        ros = ROSConnection.GetOrCreateInstance();

        // drone stay still
        targetPosition = transform.position;

        // subscribe to the topic to get the targetPosition
        ros.Subscribe<PointMsg>(
            "/drone/target_position",
            ReceiveTargetPosition
        );

        Debug.Log("Drone ROS subscriber started.");
    }

    private void Update()
    {
        // move the drone to the target position
        transform.position = Vector3.MoveTowards(
            transform.position,
            targetPosition,
            moveSpeed * Time.deltaTime
        );
    }

    private void ReceiveTargetPosition(PointMsg message)
    {
        targetPosition = new Vector3(
            (float)message.x,
            (float)message.z,
            (float)message.y
        );

        Debug.Log(
            $"New target received: {targetPosition}"
        );
    }
}