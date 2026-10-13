using System;
using System.IO;
using System.Globalization;
using System.Linq;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Editor-only physical evidence. Package ground truth never enters ROS guidance.</summary>
[InitializeOnLoad]
public static class CameraPickupTrial
{
    private const string ActiveKey = "CameraPickupTrial.Active";
    private const string SceneSetupKey = "CameraPickupTrial.SceneSetup";
    private const string WarehouseScene = "Assets/Scenes/MainWarehouse.unity";
    private static double TrialTimeout => DeliveryTrial ? 330 : 150;
    private static bool DeliveryTrial => Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_MISSION") == "delivery";
    private static bool StartFromDroneStart => DeliveryTrial && Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_START") != "observation";
    private const double SampleInterval = 0.1;
    private static readonly string Output = Path.GetFullPath(
        Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_OUTPUT") ??
        Path.Combine(Application.dataPath, "../../ROS1/build/camera-pickup/physical-trial"));

    [Serializable] private class SavedScenes { public SceneSetup[] scenes; }
    [Serializable] private class Guidance
    {
        public string phase, reason;
        public float[] center, target;
        public float speed;
        public double stamp;
    }
    [Serializable] private class Manifest
    {
        public string scenario, mission, position_frame = "warehouse_ros";
        public bool start_from_drone_start;
        public float[] start_position, observation_position;
        public float hover_time = 1;
        public float[] zone_min, zone_max;
        public float settled_center_height;
        public double started, timeout;
        public float lift_distance = 1, tolerance = 0.05f, hold_grace = 0.5f;
        public PackageSetup[] packages;
    }
    [Serializable] private class PackageSetup
    {
        public string id;
        public float[] ros_position, unity_euler_degrees;
    }
    [Serializable] private class Sample
    {
        public double stamp, status_age;
        public string phase, reason, selected_package, held_package, failure;
        public float[] drone_position, package_position;
        public bool attached, holding, released;
        public float[] package_bounds_min, package_bounds_max;
        public float linear_speed, angular_speed;
        public string[] collisions;
    }

    private static PackageHandling drone;
    private static DeliveryPackage[] packages;
    private static DeliveryPackage selectedPackage;
    private static StreamWriter history;
    private static double started, nextSample, terminalSince;
    private static Guidance guidance;
    private static string failure;
    private static bool finishing;

    static CameraPickupTrial()
    {
        EditorApplication.playModeStateChanged += OnPlayMode;
        EditorApplication.update += Update;
    }

    /// <summary>Compile/setup check only: no Play Mode, ROS commands, or pickup.</summary>
    public static void VerifySetup()
    {
        RequireCleanBatchEditor();
        SceneSetup[] original = EditorSceneManager.GetSceneManagerSetup();
        SavedScenes roundTrip = JsonUtility.FromJson<SavedScenes>(JsonUtility.ToJson(new SavedScenes { scenes = original }));
        if (roundTrip.scenes == null || roundTrip.scenes.Length != original.Length ||
            original.Where((scene, index) => scene.path != roundTrip.scenes[index].path ||
                scene.isLoaded != roundTrip.scenes[index].isLoaded || scene.isActive != roundTrip.scenes[index].isActive).Any())
            throw new InvalidOperationException("Scene restoration state did not serialize correctly.");
        try
        {
            EditorSceneManager.OpenScene(WarehouseScene);
            FindSceneObjects();
            if (drone.packageMount == null || drone.pickupDetector == null || drone.gripper == null)
                throw new InvalidOperationException("PackageHandling requires mount, detector, and gripper references.");
        }
        finally
        {
            RestoreScenes(original);
        }
        Debug.Log("CAMERA_PICKUP_EVALUATOR_SETUP_VERIFIED packages=" + packages.Length);
    }

    /// <summary>Inspect scene geometry without enabling movement or saving changes.</summary>
    public static void VerifyDeliverySetup()
    {
        RequireCleanBatchEditor();
        SceneSetup[] original = EditorSceneManager.GetSceneManagerSetup();
        try
        {
            EditorSceneManager.OpenScene(WarehouseScene);
            FindSceneObjects();
            Renderer zone = GameObject.Find("SortingZoneAVisual").GetComponent<Renderer>();
            Physics.SyncTransforms();
            Debug.Log("DELIVERY_ZONE_BOUNDS center=" + zone.bounds.center + " size=" + zone.bounds.size);
            Vector3 start = GameObject.Find("DroneStart").transform.position;
            Vector3[] route = { start, new Vector3(start.x, 6, start.z), new Vector3(34, 6, 25),
                new Vector3(32.8f, 2.65f, 24), new Vector3(32.8f, 1.25f, 24), new Vector3(32.8f, 6, 24),
                new Vector3(28, 6, 55), new Vector3(28, 1.4f, 55) };
            for (int segment = 0; segment < route.Length - 1; segment++)
            {
                // Conservative envelope covers the drone body and suspended package.
                Vector3 offset = new Vector3(0, -0.45f, 0);
                Vector3 halfSize = new Vector3(0.6f, 0.8f, 0.65f);
                Vector3 delta = route[segment + 1] - route[segment];
                string[] obstacles = Physics.BoxCastAll(route[segment] + offset, halfSize,
                    delta.normalized, Quaternion.identity, delta.magnitude, ~0, QueryTriggerInteraction.Ignore)
                    .Where(hit => !hit.collider.transform.IsChildOf(drone.transform) &&
                        hit.collider.GetComponentInParent<DeliveryPackage>() == null)
                    .Select(hit => hit.collider.name).Distinct().ToArray();
                Debug.Log("DELIVERY_ROUTE_SEGMENT " + segment + " obstacles=" + string.Join(",", obstacles));
                if (obstacles.Length != 0)
                    throw new InvalidOperationException("Configured delivery route intersects active scene colliders.");
            }
        }
        finally { RestoreScenes(original); }
    }

    /// <summary>Batch evidence capture for the dedicated camera pickup or delivery launch.</summary>
    public static void Run()
    {
        RequireCleanBatchEditor();
        if (Output.StartsWith(Path.GetFullPath(Application.dataPath) + Path.DirectorySeparatorChar))
            throw new InvalidOperationException("Trial evidence must be outside Unity/Assets.");
        Directory.CreateDirectory(Output);
        string[] previousEvidence = { "manifest.json", "unity-samples.jsonl", "COMPLETE.txt", "harness-failure.txt" };
        if (previousEvidence.Any(name => File.Exists(Path.Combine(Output, name))))
            throw new InvalidOperationException("Use a fresh output directory for each pickup trial.");
        SessionState.SetString(SceneSetupKey, JsonUtility.ToJson(new SavedScenes {
            scenes = EditorSceneManager.GetSceneManagerSetup()
        }));
        EditorSceneManager.OpenScene(WarehouseScene);
        SessionState.SetBool(ActiveKey, true);
        EditorApplication.EnterPlaymode();
    }

    private static void RequireCleanBatchEditor()
    {
        if (!Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode ||
            Enumerable.Range(0, SceneManager.sceneCount).Any(index => SceneManager.GetSceneAt(index).isDirty))
            throw new InvalidOperationException("Use a separate batch editor with clean scenes outside Play Mode.");
    }

    private static void FindSceneObjects()
    {
        PackageHandling[] drones = UnityEngine.Object.FindObjectsByType<PackageHandling>(FindObjectsSortMode.None);
        packages = UnityEngine.Object.FindObjectsByType<DeliveryPackage>(FindObjectsSortMode.None)
            .OrderBy(package => package.transform.position.x).ToArray();
        if (drones.Length != 1 || packages.Length != 3)
            throw new InvalidOperationException("Pickup trial requires one delivery drone and three demo packages.");
        drone = drones[0];
    }

    private static void OnPlayMode(PlayModeStateChange state)
    {
        if (!SessionState.GetBool(ActiveKey, false)) return;
        if (state == PlayModeStateChange.EnteredPlayMode)
        {
            try
            {
                FindSceneObjects();
                string scenario = Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_SCENARIO") ?? "baseline";
                if (scenario != "baseline" && scenario != "changed")
                    throw new InvalidOperationException("Scenario must be baseline or changed.");
                drone.transform.SetPositionAndRotation(StartFromDroneStart ? GameObject.Find("DroneStart").transform.position : new Vector3(34, 6, 25), Quaternion.identity);
                for (int index = 0; index < packages.Length; index++)
                {
                    Vector3 position = new Vector3(32.8f + index * 1.15f, 0.44f, 24 + index * 0.8f);
                    if (scenario == "changed") position += new Vector3(0.25f * Mathf.Sin(index + 1), 0, 0.3f * Mathf.Cos(index + 1));
                    packages[index].transform.SetPositionAndRotation(position,
                        Quaternion.Euler(0, scenario == "changed" ? 25 + index * 35 : 0, 0));
                }
                Physics.SyncTransforms();
                started = UtcNow();
                float liftDistance = float.Parse(Environment.GetEnvironmentVariable("DRONE_PICKUP_LIFT_DISTANCE") ?? "1",
                    CultureInfo.InvariantCulture);
                if (float.IsNaN(liftDistance) || float.IsInfinity(liftDistance) || liftDistance <= 0.05f)
                    throw new InvalidOperationException("Lift distance must be finite and greater than the 5 cm tolerance.");
                nextSample = started;
                terminalSince = 0;
                history = new StreamWriter(new FileStream(Path.Combine(Output, "unity-samples.jsonl"), FileMode.CreateNew));
                Bounds zone = GameObject.Find("SortingZoneAVisual").GetComponent<Renderer>().bounds;
                if (!Physics.Raycast(new Vector3(zone.center.x, 2, zone.center.z), Vector3.down,
                    out RaycastHit floor, 3, ~0, QueryTriggerInteraction.Ignore))
                    throw new InvalidOperationException("No physical floor beneath SortingZoneA.");
                float halfPackageHeight = packages[0].GetComponent<BoxCollider>().bounds.extents.y;
                File.WriteAllText(Path.Combine(Output, "manifest.json"), JsonUtility.ToJson(new Manifest {
                    scenario = scenario, started = started, lift_distance = liftDistance,
                    start_from_drone_start = StartFromDroneStart,
                    start_position = RosPosition(drone.transform.position),
                    observation_position = new float[] { 34, 25, 6 },
                    mission = DeliveryTrial ? "delivery" : "pickup", timeout = TrialTimeout,
                    zone_min = RosPosition(zone.min), zone_max = RosPosition(zone.max),
                    settled_center_height = floor.point.y + halfPackageHeight,
                    packages = packages.Select(package => new PackageSetup {
                        id = PackageId(package), ros_position = RosPosition(package.transform.position),
                        unity_euler_degrees = new[] { package.transform.eulerAngles.x, package.transform.eulerAngles.y, package.transform.eulerAngles.z }
                    }).ToArray()
                }, true));
                Debug.Log("CAMERA_PICKUP_TRIAL_READY " + Output);
            }
            catch (Exception exception) { Fail(exception); }
        }
        if (state == PlayModeStateChange.EnteredEditMode)
        {
            history?.Dispose();
            history = null;
            SessionState.SetBool(ActiveKey, false);
            SavedScenes original = JsonUtility.FromJson<SavedScenes>(SessionState.GetString(SceneSetupKey, ""));
            RestoreScenes(original.scenes);
            // Completion means evidence capture/restoration finished, not physical success.
            File.WriteAllText(Path.Combine(Output, "COMPLETE.txt"), "Evaluate the saved physical evidence.\n");
            EditorApplication.Exit(File.Exists(Path.Combine(Output, "harness-failure.txt")) ? 1 : 0);
        }
    }

    private static void Update()
    {
        if (!SessionState.GetBool(ActiveKey, false) || !EditorApplication.isPlaying || finishing || history == null) return;
        double now = UtcNow();
        if (now < nextSample) return;
        nextSample = now + SampleInterval;
        try
        {
            string latest = Path.Combine(Output, "latest-guidance.json");
            if (File.Exists(latest))
            {
                Guidance status = JsonUtility.FromJson<Guidance>(File.ReadAllText(latest));
                if (status != null && status.stamp >= started && status.stamp <= now)
                {
                    if (guidance == null && status.phase != (StartFromDroneStart ? "ASCEND_TO_OBSERVATION" : "OBSERVE"))
                        throw new InvalidOperationException("Trial requires a fresh mission with the configured start; check leftover control publishers.");
                    guidance = status;
                }
            }
            if (selectedPackage == null && guidance?.phase == "APPROACH") AssociateSelectedPackage();
            // Trial-only sensor fault; operational guidance receives no scene ground truth.
            if (guidance?.phase == "APPROACH" &&
                Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_DISABLE_CAMERA") == "true")
            {
                DroneCameraPublisher publisher = drone.GetComponent<DroneCameraPublisher>();
                if (publisher != null && publisher.enabled)
                {
                    publisher.enabled = false;
                    File.WriteAllText(Path.Combine(Output, "camera-disabled.txt"), now.ToString(CultureInfo.InvariantCulture));
                    Debug.Log("CAMERA_PICKUP_TRIAL_CAMERA_DISABLED");
                }
            }
            RecordSample(now);
            if (now - started > TrialTimeout)
            {
                failure = "trial_timeout";
                // Trial-only sensor fault; operational guidance receives no scene ground truth.
            if (guidance?.phase == "APPROACH" &&
                Environment.GetEnvironmentVariable("DRONE_PICKUP_TRIAL_DISABLE_CAMERA") == "true")
            {
                DroneCameraPublisher publisher = drone.GetComponent<DroneCameraPublisher>();
                if (publisher != null && publisher.enabled)
                {
                    publisher.enabled = false;
                    File.WriteAllText(Path.Combine(Output, "camera-disabled.txt"), now.ToString(CultureInfo.InvariantCulture));
                    Debug.Log("CAMERA_PICKUP_TRIAL_CAMERA_DISABLED");
                }
            }
            RecordSample(now);
                Finish();
            }
            else if (guidance?.phase == (DeliveryTrial ? "COMPLETE" : "VERIFIED") || guidance?.phase == "FAILED")
            {
                if (terminalSince == 0) terminalSince = now;
                // Capture continued attachment/hold for one second after the terminal phase.
                if (now - terminalSince >= 1) Finish();
            }
        }
        catch (Exception exception) { Fail(exception); }
    }

    private static void AssociateSelectedPackage()
    {
        if (guidance.center == null || guidance.center.Length != 3 || guidance.center.Any(value => float.IsNaN(value) || float.IsInfinity(value)))
            throw new InvalidOperationException("Selected camera center is invalid.");
        Vector3 center = new Vector3(guidance.center[0], guidance.center[2], guidance.center[1]);
        DeliveryPackage[] matches = packages.Where(package => Vector3.Distance(package.transform.position, center) <= 0.1f).ToArray();
        if (matches.Length != 1)
            throw new InvalidOperationException("Selected camera center does not identify exactly one package within 10 cm.");
        selectedPackage = matches[0];
    }

    private static void RecordSample(double now)
    {
        DeliveryPackage held = drone.HeldPackage;
        bool attached = selectedPackage != null && held == selectedPackage && selectedPackage.Owner == drone &&
            selectedPackage.transform.parent == drone.packageMount;
        BoxCollider packageCollider = selectedPackage == null ? null : selectedPackage.GetComponent<BoxCollider>();
        Rigidbody packageBody = selectedPackage == null ? null : selectedPackage.GetComponent<Rigidbody>();
        history.WriteLine(JsonUtility.ToJson(new Sample {
            stamp = now, status_age = guidance == null ? now - started : now - guidance.stamp,
            phase = guidance?.phase ?? "WAITING_FOR_GUIDANCE", reason = guidance?.reason,
            selected_package = PackageId(selectedPackage), held_package = PackageId(held),
            drone_position = RosPosition(drone.transform.position),
            package_position = RosPosition(selectedPackage == null ? Vector3.zero : selectedPackage.transform.position),
            attached = attached, holding = guidance != null && guidance.speed == 0, failure = failure,
            released = selectedPackage != null && held == null && selectedPackage.Owner == null &&
                selectedPackage.transform.parent != drone.packageMount,
            package_bounds_min = RosPosition(packageCollider == null ? Vector3.zero : packageCollider.bounds.min),
            package_bounds_max = RosPosition(packageCollider == null ? Vector3.zero : packageCollider.bounds.max),
            linear_speed = packageBody == null ? 0 : packageBody.linearVelocity.magnitude,
            angular_speed = packageBody == null ? 0 : packageBody.angularVelocity.magnitude,
            collisions = DeliveryTrial ? CollisionNames(attached) : Array.Empty<string>()
        }));
        history.Flush();
    }

    private static string[] CollisionNames(bool attached)
    {
        // Evaluate actual penetration because transform-driven motion has no collision response.
        Collider[] moving = drone.GetComponentsInChildren<Collider>();
        if (attached) moving = moving.Concat(selectedPackage.GetComponentsInChildren<Collider>()).Distinct().ToArray();
        return moving.Where(body => body.enabled && !body.isTrigger).SelectMany(body =>
            Physics.OverlapBox(body.bounds.center, body.bounds.extents, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore)
                .Where(other => !other.transform.IsChildOf(drone.transform) && other != body &&
                    Physics.ComputePenetration(body, body.transform.position, body.transform.rotation,
                        other, other.transform.position, other.transform.rotation, out _, out float depth) && depth > 0.01f)
                .Select(other => other.name)).Distinct().ToArray();
    }

    private static float[] RosPosition(Vector3 position) => new[] { position.x, position.z, position.y };
    private static double UtcNow() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

    private static string PackageId(DeliveryPackage package)
    {
        if (package == null) return "";
        int index = Array.IndexOf(packages, package);
        return index < 0 ? "untracked-package" : "package-" + (index + 1);
    }

    private static void RestoreScenes(SceneSetup[] original)
    {
        // A fresh batch editor can start with no loaded/active scene. Unity cannot
        // restore that empty setup; return it to a clean, unsaved empty scene.
        if (original.Any(scene => scene.isLoaded) && original.Any(scene => scene.isActive))
            EditorSceneManager.RestoreSceneManagerSetup(original);
        else
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
    }

    private static void Fail(Exception exception)
    {
        failure = exception.Message;
        Debug.LogException(exception);
        File.WriteAllText(Path.Combine(Output, "harness-failure.txt"), exception.ToString());
        Finish();
    }

    private static void Finish()
    {
        finishing = true;
        history?.Dispose();
        history = null;
        EditorApplication.ExitPlaymode();
    }
}
