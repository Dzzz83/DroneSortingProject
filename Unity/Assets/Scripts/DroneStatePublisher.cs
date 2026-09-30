using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneStatePublisher : MonoBehaviour
{
    [SerializeField]
    private string topicName = "/drone/current_position";

    [SerializeField]
    private float publishRate = 0.2f;

    private ROSConnection ros;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishRate)
        {
            Vector3 unityPosition = transform.position;

            PointMsg position = new PointMsg(
                unityPosition.x,
                unityPosition.z,
                unityPosition.y
            );

            ros.Publish(topicName, position);

            timeElapsed = 0f;
        }
    }
}