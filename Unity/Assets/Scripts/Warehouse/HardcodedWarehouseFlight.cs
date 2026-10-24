using System.Collections;
using DroneDelivery;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[RequireComponent(typeof(WarehouseCourse))]
public sealed class HardcodedWarehouseFlight : MonoBehaviour
{
    public bool IsFlying { get; private set; }
    public string Status { get; private set; } = "Ready";
    WarehouseCourse course;
    DroneCommandSubscriber commands;
    PackageActionBridge actions;
    bool ownsControl, previousCommands, previousActions;

    void Awake()
    {
        course = GetComponent<WarehouseCourse>();
        commands = GetComponent<DroneCommandSubscriber>();
        actions = GetComponent<PackageActionBridge>();
        // This local demonstration does not need a ROS server.
        if (enabled) ROSConnection.GetOrCreateInstance().ConnectOnStart = false;
    }

    void Start() { StartFlight(); }

    public void StartFlight()
    {
        course.ResetTrial();
        previousCommands = commands.enabled;
        previousActions = actions.enabled;
        commands.enabled = actions.enabled = false;
        ownsControl = IsFlying = true;
        StartCoroutine(Fly());
    }

    public void CancelFlight()
    {
        StopAllCoroutines();
        IsFlying = false;
        Status = "Ready";
        if (!ownsControl) return;
        commands.enabled = previousCommands;
        actions.enabled = previousActions;
        ownsControl = false;
    }

    void OnDisable() { CancelFlight(); }

    IEnumerator Fly()
    {
        Status = "Flying to pickup in A";
        yield return Move(new Vector3(34, 3, 50));
        yield return Move(new Vector3(34, 3, 55));
        Vector3 mountOffset = course.drone.packageMount.position - transform.position;
        yield return Move(course.package.transform.position + Vector3.up * .08f - mountOffset);
        if (!IsFlying) yield break;
        Status = "Picking up package";
        if (!course.drone.PickUpPackage()) { Fail("Pickup could not start"); yield break; }
        float deadline = Time.time + 4;
        while (course.drone.IsBusy && Time.time < deadline) yield return null;
        if (course.drone.HeldPackage != course.package) { Fail("Pickup failed"); yield break; }

        Status = "Carrying package to B";
        // shortcut: fixed route for the saved static course; use ROS planning if the layout changes.
        Vector3[] route = {
            new Vector3(34,3,55), new Vector3(34,3,71),
            new Vector3(25,3,71), new Vector3(25,3,87),
            new Vector3(43,3,87), new Vector3(43,3,103),
            new Vector3(34,3,103), new Vector3(34,3,119.5f)
        };
        foreach (var point in route) yield return Move(point);
        if (!IsFlying) yield break;
        Status = "Releasing package in B";
        yield return Move(new Vector3(34, .65f - mountOffset.y, 119.5f));
        if (!IsFlying) yield break;
        if (!course.drone.DropPackage()) { Fail("Release failed"); yield break; }
        deadline = Time.time + 6;
        while (!course.Delivered && Time.time < deadline) yield return null;
        if (!course.Delivered) { Fail("Package did not settle inside B"); yield break; }
        yield return Move(new Vector3(34, 3, 119.5f));
        if (!IsFlying) yield break;
        IsFlying = false;
        Status = "Delivered in Warehouse B";
        Debug.Log("HARDCODED_FLIGHT_DELIVERED");
    }

    IEnumerator Move(Vector3 destination)
    {
        while (IsFlying && Vector3.Distance(transform.position, destination) > .001f)
        {
            yield return new WaitForFixedUpdate();
            Vector3 next = Vector3.MoveTowards(transform.position, destination, 3f * Time.fixedDeltaTime);
            course.ObserveCommand(next);
            if (course.CollisionObserved) { Fail("Contact: " + course.ContactName); yield break; }
            transform.position = next;
            Physics.SyncTransforms();
        }
    }

    void Fail(string reason)
    {
        IsFlying = false;
        Status = "Stopped - " + reason;
        Debug.LogWarning("HARDCODED_FLIGHT_STOPPED: " + reason);
    }
}
