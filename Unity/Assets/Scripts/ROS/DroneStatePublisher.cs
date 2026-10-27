using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneStatePublisher : MonoBehaviour
{
    [SerializeField]
    private string topicName = DroneRosTopics.CurrentPosition;

    [SerializeField]
    private float publishRate = 0.05f;

    private ROSConnection ros;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName);
    }

    void LateUpdate()
    {
        // Wall-clock cadence; send one real pose, never catch-up duplicates.
        timeElapsed += Time.unscaledDeltaTime;

        if (timeElapsed >= publishRate)
        {
            Vector3 unityPosition = transform.position;

            PointMsg position = new PointMsg(
                unityPosition.x,
                unityPosition.z,
                unityPosition.y
            );

            ros.Publish(topicName, position);

            // Keep fractional frame time without sending catch-up duplicates.
            timeElapsed %= publishRate;
        }
    }
}