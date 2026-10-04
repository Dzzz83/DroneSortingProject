using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class DroneObstacleSensorPublisher : MonoBehaviour
{
    [Header("ROS")]
    [SerializeField]
    private string topicName = DroneRosTopics.ObstacleDistances;

    [SerializeField]
    private string headingTopicName = DroneRosTopics.TravelHeading;

    [Header("Sensor Settings")]
    [SerializeField]
    private float maxDistance = 5.0f;

    [SerializeField]
    private float sensorRadius = 0.35f;

    [SerializeField]
    private float verticalProbeOffset = 0.8f;

    [SerializeField]
    private float publishInterval = 0.1f;

    private ROSConnection ros;
    private float elapsedTime;

    // Default direction before ROS heading arrives.
    private Vector3 forwardDirection = Vector3.right;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<Float32MultiArrayMsg>(
            topicName
        );

        ros.Subscribe<Vector3Msg>(
            headingTopicName,
            HeadingCallback
        );
    }

    void Update()
    {
        elapsedTime += Time.deltaTime;

        if (elapsedTime < publishInterval)
            return;

        PublishSensorReadings();

        elapsedTime = 0.0f;
    }

    private void HeadingCallback(Vector3Msg message)
    {
        // ROS:
        // X -> Unity X
        // Y -> Unity Z
        //
        // ROS Z is vertical, so it is ignored
        // for the horizontal sensor heading.

        Vector3 newDirection = new Vector3(
            (float)message.x,
            0.0f,
            (float)message.y
        );

        if (newDirection.sqrMagnitude > 0.000001f)
        {
            forwardDirection =
                newDirection.normalized;
        }
    }

    private void PublishSensorReadings()
    {
        Vector3 center = transform.position;
        Vector3 upDirection = Vector3.up;
        Vector3 downDirection = Vector3.down;

        Vector3 rightDirection = Vector3.Cross(
            upDirection,
            forwardDirection
        ).normalized;

        Vector3 leftDirection = -rightDirection;

        Vector3 upperOrigin =
            center + upDirection * verticalProbeOffset;

        Vector3 lowerOrigin =
            center - upDirection * verticalProbeOffset;

        float front =
            MeasureDistance(center, forwardDirection);

        float upperFront =
            MeasureDistance(upperOrigin, forwardDirection);

        float lowerFront =
            MeasureDistance(lowerOrigin, forwardDirection);

        float left =
            MeasureDistance(center, leftDirection);

        float right =
            MeasureDistance(center, rightDirection);

        float up =
            MeasureDistance(center, upDirection);

        float down =
            MeasureDistance(center, downDirection);

        Float32MultiArrayMsg message =
            new Float32MultiArrayMsg();

        message.data = new float[]
        {
            front,
            upperFront,
            lowerFront,
            left,
            right,
            up,
            down
        };

        ros.Publish(topicName, message);

        DrawSensors(
            center,
            upperOrigin,
            lowerOrigin,
            forwardDirection,
            leftDirection,
            rightDirection
        );
    }

    private float MeasureDistance(
        Vector3 origin,
        Vector3 direction
    )
    {
        RaycastHit[] hits = Physics.SphereCastAll(
            origin,
            sensorRadius,
            direction.normalized,
            maxDistance,
            Physics.DefaultRaycastLayers,
            QueryTriggerInteraction.Ignore
        );

        float nearestDistance =
            float.PositiveInfinity;

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider.transform.root == transform.root)
                continue;

            if (hit.distance < nearestDistance)
            {
                nearestDistance = hit.distance;
            }
        }

        if (float.IsPositiveInfinity(nearestDistance))
        {
            return -1.0f;
        }

        return nearestDistance;
    }

    private void DrawSensors(
        Vector3 center,
        Vector3 upperOrigin,
        Vector3 lowerOrigin,
        Vector3 front,
        Vector3 left,
        Vector3 right
    )
    {
        Debug.DrawRay(
            center,
            front * maxDistance
        );

        Debug.DrawRay(
            upperOrigin,
            front * maxDistance
        );

        Debug.DrawRay(
            lowerOrigin,
            front * maxDistance
        );

        Debug.DrawRay(
            center,
            left * maxDistance
        );

        Debug.DrawRay(
            center,
            right * maxDistance
        );

        Debug.DrawRay(
            center,
            Vector3.up * maxDistance
        );

        Debug.DrawRay(
            center,
            Vector3.down * maxDistance
        );
    }
}