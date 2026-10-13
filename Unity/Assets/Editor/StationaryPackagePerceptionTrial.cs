using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Temporary test poses and independent collider ground truth; never mission input.</summary>
[InitializeOnLoad]
public static class StationaryPackagePerceptionTrial
{
    private const string Flag = "StationaryPackagePerceptionTrial.Active";
    private static readonly string Output = Path.GetFullPath(Environment.GetEnvironmentVariable("DRONE_PERCEPTION_TRIAL_OUTPUT") ??
        Path.Combine(Application.dataPath, "../../ROS1/build/stationary-perception"));
    private static readonly string[] Cases = { "three_packages", "changed_placements", "approach", "partial_occlusion", "empty", "missing_camera" };
    private static readonly List<Observation> Observations = new List<Observation>();
    private static int currentCase = -1;
    private static double started, caseStamp;
    private static uint lastSequence = uint.MaxValue;
    private static PackageHandling drone;
    private static DeliveryPackage[] packages;
    private static Camera camera;
    private static DroneRgbCamera sensor;
    private static GameObject blocker;
    private static DroneCameraPublisher publisher;

    [Serializable] private class Stamp { public long secs, nsecs; }
    [Serializable] private class Grasp { public bool ready; public float[] center; }
    [Serializable] private class Candidate { public float[] position; public int[] pixel; public Grasp grasp; }
    [Serializable] private class Result { public string state; public uint sequence; public Stamp stamp; public Candidate[] candidates; }
    [Serializable] private class Observation
    {
        public string testCase, state;
        public uint sequence;
        public int candidates, matchedPackages;
        public float[] errorsMeters;
        public float[] centerErrorsMeters;
        public string failure;
    }
    [Serializable] private class Report { public Observation[] observations; }

    static StationaryPackagePerceptionTrial()
    {
        EditorApplication.playModeStateChanged += OnPlayMode;
        EditorApplication.update += Update;
    }

    public static void Run()
    {
        if (!Application.isBatchMode)
            throw new InvalidOperationException("Run this automated trial in a separate batch-mode editor.");
        if (EditorApplication.isPlayingOrWillChangePlaymode ||
            Enumerable.Range(0, SceneManager.sceneCount).Any(i => SceneManager.GetSceneAt(i).isDirty))
            throw new InvalidOperationException("Stationary trial requires clean scenes outside Play Mode.");
        Directory.CreateDirectory(Output);
        EditorSceneManager.OpenScene("Assets/Scenes/MainWarehouse.unity");
        SessionState.SetBool(Flag, true);
        EditorApplication.EnterPlaymode();
    }

    private static void OnPlayMode(PlayModeStateChange state)
    {
        if (!SessionState.GetBool(Flag, false)) return;
        if (state == PlayModeStateChange.EnteredPlayMode)
        {
            drone = UnityEngine.Object.FindAnyObjectByType<PackageHandling>();
            packages = UnityEngine.Object.FindObjectsByType<DeliveryPackage>().OrderBy(p => p.transform.position.x).ToArray();
            sensor = drone.GetComponentInChildren<DroneRgbCamera>();
            camera = sensor.SensorCamera;
            publisher = drone.GetComponent<DroneCameraPublisher>();
            foreach (DeliveryPackage package in packages)
                foreach (Rigidbody body in package.GetComponentsInChildren<Rigidbody>()) body.isKinematic = true;
            blocker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            blocker.name = "StationaryPerceptionTestOccluder";
            blocker.GetComponent<Renderer>().sharedMaterial = new Material(Shader.Find("HDRP/Lit"));
            blocker.SetActive(false);
            Configure(0);
        }
        if (state == PlayModeStateChange.EnteredEditMode)
        {
            SessionState.SetBool(Flag, false);
            // Play Mode edits are discarded; reload the original scene without saving.
            EditorSceneManager.OpenScene("Assets/Scenes/MainWarehouse.unity");
            EditorApplication.Exit(0);
        }
    }

    private static void Configure(int index)
    {
        currentCase = index;
        started = EditorApplication.timeSinceStartup;
        caseStamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        publisher.enabled = index != 5;
        drone.transform.SetPositionAndRotation(new Vector3(34f, index == 2 ? 2.8f : 6f, 25f), Quaternion.identity);
        for (int p = 0; p < packages.Length; p++)
        {
            packages[p].gameObject.SetActive(index < 4);
            Vector3 position = new Vector3(32.8f + p * 1.15f, 0.44f, 24f + p * 0.8f);
            if (index == 1) position += new Vector3(0.25f * Mathf.Sin(p + 1), 0, 0.3f * Mathf.Cos(p + 1));
            packages[p].transform.SetPositionAndRotation(position, Quaternion.Euler(0, index == 1 ? 25 + p * 35 : 0, 0));
        }
        blocker.SetActive(index == 3);
        if (index == 3)
        {
            blocker.transform.SetPositionAndRotation(Vector3.Lerp(camera.transform.position, packages[0].transform.position, 0.5f), camera.transform.rotation);
            blocker.transform.localScale = new Vector3(0.15f, 0.8f, 0.08f);
        }
        Physics.SyncTransforms();
        Debug.Log("STATIONARY_PERCEPTION_CASE " + Cases[index]);
    }

    private static void Update()
    {
        if (!SessionState.GetBool(Flag, false) || !EditorApplication.isPlaying || currentCase < 0) return;
        double elapsed = EditorApplication.timeSinceStartup - started;
        if (elapsed > 2 && File.Exists(Path.Combine(Output, "latest.json")))
        {
            Result result = JsonUtility.FromJson<Result>(File.ReadAllText(Path.Combine(Output, "latest.json")));
            if (currentCase == 5 && result.state == "UNAVAILABLE")
            {
                if (!Observations.Any(o => o.testCase == Cases[5]))
                    Observations.Add(new Observation { testCase = Cases[5], state = result.state });
            }
            else if (result.stamp != null && result.stamp.secs + result.stamp.nsecs / 1e9 > caseStamp + 1 && result.sequence != lastSequence)
            {
                lastSequence = result.sequence;
                Evaluate(result);
            }
        }
        if (elapsed < (currentCase == 0 ? 20 : 10)) return;
        if (currentCase + 1 < Cases.Length) Configure(currentCase + 1);
        else
        {
            File.WriteAllText(Path.Combine(Output, "unity-evaluation.json"), JsonUtility.ToJson(new Report { observations = Observations.ToArray() }, true));
            currentCase = -1;
            EditorApplication.ExitPlaymode();
        }
    }

    private static void Evaluate(Result result)
    {
        var errors = new List<float>();
        var centerErrors = new List<float>();
        var matched = new HashSet<DeliveryPackage>();
        string failure = null;
        foreach (Candidate candidate in result.candidates ?? Array.Empty<Candidate>())
        {
            // Sensor dimensions differ from the editor's Game View dimensions.
            Ray ray = camera.ViewportPointToRay(new Vector3((candidate.pixel[0] + 0.5f) / sensor.ImageWidth,
                1f - (candidate.pixel[1] + 0.5f) / sensor.ImageHeight, 0));
            if (Physics.Raycast(ray, out RaycastHit hit, 20f) && hit.collider.GetComponentInParent<DeliveryPackage>() is DeliveryPackage package)
            {
                matched.Add(package);
                Vector3 point = new Vector3(candidate.position[0], candidate.position[2], candidate.position[1]);
                errors.Add(Vector3.Distance(point, hit.point));
                if (candidate.grasp != null && candidate.grasp.ready)
                {
                    float[] center = candidate.grasp.center;
                    centerErrors.Add(Vector3.Distance(new Vector3(center[0], center[2], center[1]), package.transform.position));
                }
            }
            else failure = "Candidate pixel does not hit a package collider";
        }
        Observations.Add(new Observation { testCase = Cases[currentCase], state = result.state, sequence = result.sequence,
            candidates = result.candidates?.Length ?? 0, matchedPackages = matched.Count, errorsMeters = errors.ToArray(),
            centerErrorsMeters = centerErrors.ToArray(), failure = failure });
    }
}
