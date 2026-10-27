using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Legacy point executor, or opt-in ordered, expiring ROS-approved segments.</summary>
public class DroneCommandSubscriber : MonoBehaviour
{
    const int MaximumQueuedSegments = 8;
    const float MaximumQueuedDistance = .255f;
    const float MatchTolerance = .0002f;
    readonly Queue<Segment> pending = new Queue<Segment>();
    ROSConnection ros;
    WarehouseCourse course;
    bool trajectoryMode, requireRestart = true;
    int lastSequence;
    Vector3 queuedEnd;
    float statusElapsed;

    [Serializable]
    class TrajectoryStatus
    {
        public int reset_sequence, sequence;
        public bool stopped;
        public long stamp_ms;
        public Vector3 position;
    }

    [Serializable]
    class ApprovedCommand
    {
        public int reset_sequence, sequence;
        public bool stop, restart;
        public long issued_ms, expires_ms;
        public float speed;
        public Vector3 start, end; // ROS warehouse X/Y horizontal, Z vertical
    }

    struct Segment
    {
        public Vector3 End;
        public float Speed;
        public long ExpiryMs;
    }

    void Start()
    {
        course = GetComponent<WarehouseCourse>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointMsg>(DroneRosTopics.CommandPosition, ReceivePosition);
        ros.Subscribe<StringMsg>(DroneRosTopics.ApprovedTrajectory, ReceiveApprovedTrajectory);
        ros.RegisterPublisher<StringMsg>(DroneRosTopics.TrajectoryStatus, queue_size: 1);
    }

    void ReceivePosition(PointMsg message)
    {
        // The old package and heuristic workflows retain immediate point commands.
        if (!enabled || trajectoryMode) return;
        Vector3 target = new Vector3((float)message.x, (float)message.z, (float)message.y);
        if (!Finite(target))
        {
            Debug.LogWarning("Ignored non-finite ROS position command.");
            return;
        }
        if (course != null) course.ObserveCommand(target);
        transform.position = target;
    }

    static Vector3 FromRos(Vector3 point) => new Vector3(point.x, point.z, point.y);
    static bool Finite(Vector3 p) => float.IsFinite(p.x) && float.IsFinite(p.y) && float.IsFinite(p.z);
    static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

    void ReceiveApprovedTrajectory(StringMsg message) => ApplyApprovedTrajectory(message.data, NowMs());

    public void ApplyApprovedTrajectory(string json, long now)
    {
        if (course == null) course = GetComponent<WarehouseCourse>();
        if (!enabled || course == null) return; // Opt-in: only the two-warehouse course.
        ApprovedCommand packet;
        try { packet = JsonUtility.FromJson<ApprovedCommand>(json); }
        catch (Exception) { StopTrajectory(); return; }
        if (packet == null || packet.reset_sequence != course.ResetSequence || packet.sequence <= lastSequence)
            return; // Never replay a pre-reset or out-of-order command.
        int previousSequence = lastSequence;
        lastSequence = packet.sequence; // Rejections must also be visible to ROS recovery.
        if (packet.stop)
        {
            lastSequence = packet.sequence;
            StopTrajectory();
            trajectoryMode = true;
            return;
        }
        Vector3 start = FromRos(packet.start), end = FromRos(packet.end);
        if (!Finite(start) || !Finite(end) || !float.IsFinite(packet.speed) ||
            packet.speed <= 0f || packet.speed > 10f ||
            packet.issued_ms > now + 50 || now - packet.issued_ms > 500 ||
            packet.expires_ms <= now || packet.expires_ms - packet.issued_ms > 200 ||
            packet.expires_ms <= packet.issued_ms ||
            Vector3.Distance(start, end) < .000001f || Vector3.Distance(start, end) > .25001f ||
            course.CollisionObserved)
        {
            StopTrajectory();
            return;
        }
        if (packet.restart)
        {
            // A restart is safe only after all previous motion was stopped and
            // ROS has re-anchored to the actual applied position from feedback.
            if (!requireRestart || pending.Count != 0 ||
                Vector3.Distance(transform.position, start) > MatchTolerance)
            {
                StopTrajectory();
                return;
            }
            lastSequence = packet.sequence;
            trajectoryMode = true;
            requireRestart = false;
            queuedEnd = start;
        }
        else if (requireRestart || packet.sequence != previousSequence + 1 ||
                 Vector3.Distance(queuedEnd, start) > .0002f)
        {
            StopTrajectory();
            return;
        }

        // Bound the sum of approved segments, not their straight-line shortcut.
        if (pending.Count >= MaximumQueuedSegments || RemainingDistance() +
            Vector3.Distance(start, end) > MaximumQueuedDistance)
        {
            StopTrajectory();
            return;
        }
        pending.Enqueue(new Segment { End = end, Speed = packet.speed, ExpiryMs = packet.expires_ms });
        queuedEnd = end;
        lastSequence = packet.sequence;
    }

    float RemainingDistance()
    {
        Vector3 start = transform.position;
        float distance = 0;
        foreach (Segment segment in pending)
        {
            distance += Vector3.Distance(start, segment.End);
            start = segment.End;
        }
        return distance;
    }

    public string TrajectoryStatusJson(long now)
    {
        Vector3 p = transform.position;
        return JsonUtility.ToJson(new TrajectoryStatus {
            reset_sequence = course.ResetSequence, sequence = lastSequence,
            stopped = requireRestart, stamp_ms = now,
            position = new Vector3(p.x, p.z, p.y) });
    }

    void Update()
    {
        AdvanceTrajectory(Time.deltaTime, NowMs());
    }

    void LateUpdate()
    {
        statusElapsed += Time.unscaledDeltaTime;
        if (course != null && ros != null && statusElapsed >= .05f)
        {
            statusElapsed %= .05f;
            ros.Publish(DroneRosTopics.TrajectoryStatus, new StringMsg(TrajectoryStatusJson(NowMs())));
        }
    }

    public void AdvanceTrajectory(float deltaTime, long now)
    {
        if (!trajectoryMode || pending.Count == 0) return;
        if (course == null || course.CollisionObserved) { StopTrajectory(); return; }
        float frameTime = Mathf.Max(0f, deltaTime);
        while (pending.Count > 0 && frameTime > 0f)
        {
            Segment segment = pending.Peek();
            if (now >= segment.ExpiryMs) { StopTrajectory(); return; }
            Vector3 current = transform.position;
            float distance = Vector3.Distance(current, segment.End);
            if (distance <= .000001f) { transform.position = segment.End; pending.Dequeue(); continue; }
            float travel = Mathf.Min(distance, segment.Speed * frameTime);
            Vector3 next = Vector3.MoveTowards(current, segment.End, travel);
            // Sweep each actual movement before moving; stop on reported contact.
            course.ObserveCommand(next);
            if (course.CollisionObserved) { StopTrajectory(); return; }
            transform.position = next;
            frameTime -= travel / segment.Speed;
            if (travel + .000001f >= distance) pending.Dequeue();
        }
    }

    void StopTrajectory()
    {
        pending.Clear();
        requireRestart = true;
        queuedEnd = transform.position;
    }

    /// <summary>Called synchronously by the scene's physical reset.</summary>
    public void ResetForTrial()
    {
        StopTrajectory();
        trajectoryMode = false;
        lastSequence = 0;
    }

    void OnDisable() => StopTrajectory();
}
