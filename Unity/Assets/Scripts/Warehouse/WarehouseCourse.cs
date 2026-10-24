using System;
using DroneDelivery;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Physical observations and reset for ROS control or the local flight demonstration.</summary>
public sealed class WarehouseCourse : MonoBehaviour
{
    public PackageHandling drone;
    public DeliveryPackage package;
    public BoxCollider deliveryVolume;
    public Collider[] packageSupports;
    public const string ObservationTopic = "/drone/course/observations";
    public const string ResetTopic = "/drone/course/reset";
    public bool Delivered { get; private set; }
    public bool CollisionObserved { get; private set; }
    public string ContactName { get; private set; } = "";
    public int ResetSequence { get; private set; }
    Vector3 droneStart, packageStart;
    Quaternion droneRotation, packageRotation;
    Rigidbody packageBody;
    BoxCollider[] flightColliders;
    ROSConnection ros;
    float settled, nextPublish;
    bool pickedUp;

    [Serializable]
    class Observation
    {
        public int reset_sequence;
        public bool package_attached, delivered, collision_observed;
        public string contact;
        public Vector3 package_position;
    }

    void Awake()
    {
        flightColliders = drone.GetComponentsInChildren<BoxCollider>();
        droneStart = drone.transform.position;
        droneRotation = drone.transform.rotation;
        packageStart = package.transform.position;
        packageRotation = package.transform.rotation;
        packageBody = package.GetComponent<Rigidbody>();
    }

    void Start()
    {
        WarehouseSetup.ConfigureDrone(drone.gameObject);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(ObservationTopic, queue_size: 1);
        ros.Subscribe<StringMsg>(ResetTopic, message => { if (message.data == "RESET") ResetTrial(); });
    }

    public void ResetTrial()
    {
        var flight = GetComponent<HardcodedWarehouseFlight>();
        if (flight != null) flight.CancelFlight();
        drone.GetComponent<PackageActionBridge>().ResetForTrial();
        // Disabling the existing handler cancels pickup and safely releases its package.
        drone.enabled = false;
        drone.enabled = true;
        drone.transform.SetPositionAndRotation(droneStart, droneRotation);
        package.transform.SetPositionAndRotation(packageStart, packageRotation);
        packageBody.linearVelocity = Vector3.zero;
        packageBody.angularVelocity = Vector3.zero;
        packageBody.WakeUp();
        Delivered = CollisionObserved = pickedUp = false;
        ContactName = "";
        settled = 0;
        ResetSequence++;
        Physics.SyncTransforms();
        Publish();
    }

    // The existing ROS position interface teleports. Sweep both physical boxes
    // before applying a command so crossing a thin wall is still observable.
    public void ObserveCommand(Vector3 target)
    {
        Physics.SyncTransforms();
        Vector3 displacement = target - drone.transform.position;
        foreach (var collider in flightColliders) CheckBox(collider, displacement, false);
        if (drone.HasPackage) CheckBox(package.GetComponent<BoxCollider>(), displacement, false);
    }

    void FixedUpdate()
    {
        Physics.SyncTransforms();
        foreach (var collider in flightColliders) CheckBox(collider, Vector3.zero, false);
        CheckBox(package.GetComponent<BoxCollider>(), Vector3.zero, true);
        pickedUp |= drone.HeldPackage == package;
        bool resting = pickedUp && package.Owner == null && ContainsPackage() &&
            packageBody.linearVelocity.sqrMagnitude < 0.0025f &&
            packageBody.angularVelocity.sqrMagnitude < 0.01f;
        settled = resting ? settled + Time.fixedDeltaTime : 0;
        Delivered = settled >= 0.5f;
        if (Time.unscaledTime >= nextPublish) { nextPublish = Time.unscaledTime + 0.1f; Publish(); }
    }

    public bool ContainsPackage()
    {
        Bounds bounds = package.GetComponent<BoxCollider>().bounds;
        // The saved delivery volume is axis-aligned; require the whole box inside it.
        return deliveryVolume.bounds.Contains(bounds.min) && deliveryVolume.bounds.Contains(bounds.max);
    }

    void CheckBox(BoxCollider box, Vector3 displacement, bool ignoreSupports)
    {
        if (!box || !box.enabled) return;
        Vector3 center = box.transform.TransformPoint(box.center);
        Vector3 half = Vector3.Scale(box.size * 0.5f, Abs(box.transform.lossyScale));
        foreach (Collider other in Physics.OverlapBox(center, half, box.transform.rotation,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (Ignore(other, ignoreSupports)) continue;
            if (Physics.ComputePenetration(box, box.transform.position, box.transform.rotation,
                    other, other.transform.position, other.transform.rotation, out _, out float depth) && depth > 0.002f)
                RecordContact(other);
        }
        if (displacement.sqrMagnitude < 0.000001f) return;
        foreach (RaycastHit hit in Physics.BoxCastAll(center, half, displacement.normalized,
                     box.transform.rotation, displacement.magnitude, Physics.DefaultRaycastLayers,
                     QueryTriggerInteraction.Ignore))
            if (!Ignore(hit.collider, ignoreSupports)) RecordContact(hit.collider);
    }

    bool Ignore(Collider other, bool ignoreSupports)
    {
        if (other.transform.IsChildOf(drone.transform) || other.transform.IsChildOf(package.transform)) return true;
        return ignoreSupports && Array.IndexOf(packageSupports, other) >= 0;
    }

    void RecordContact(Collider other) { CollisionObserved = true; ContactName = other.name; }
    static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));

    void Publish()
    {
        if (ros == null) return;
        Vector3 p = package.transform.position;
        ros.Publish(ObservationTopic, new StringMsg(JsonUtility.ToJson(new Observation {
            reset_sequence = ResetSequence, package_attached = drone.HeldPackage == package,
            delivered = Delivered, collision_observed = CollisionObserved, contact = ContactName,
            package_position = new Vector3(p.x, p.z, p.y)
        })));
    }

    void OnGUI()
    {
        Matrix4x4 previous = GUI.matrix;
        GUI.matrix = Matrix4x4.Scale(Vector3.one * Mathf.Max(1, Screen.height / 720f));
        var flight = GetComponent<HardcodedWarehouseFlight>();
        bool localFlight = flight != null && flight.enabled;
        GUI.Box(new Rect(12, 12, 420, 112), localFlight ? flight.Status : "TWO WAREHOUSE DELIVERY | ROS controls flight");
        GUI.Label(new Rect(24, 38, 400, 24), "1 Overview   2 Follow   3 Yard   |   Package: " + (drone.HasPackage ? "attached" : "free"));
        GUI.Label(new Rect(24, 62, 400, 24), "Delivery: " + (Delivered ? "CONFIRMED" : "awaiting settled package") +
            "   Contact: " + (CollisionObserved ? ContactName : "none"));
        if (GUI.Button(new Rect(24, 90, 150, 25), "Reset physical trial")) ResetTrial();
        if (localFlight && !flight.IsFlying && GUI.Button(new Rect(190, 90, 150, 25), "Start A to B flight")) flight.StartFlight();
        GUI.matrix = previous;
    }
}
