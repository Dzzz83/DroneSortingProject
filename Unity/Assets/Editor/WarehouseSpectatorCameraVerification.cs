using System;
using System.IO;
using System.Linq;
using DroneDelivery;
using Unity.Cinemachine;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

/// <summary>Checks clearance and renders previews, restoring scene poses without saving.</summary>
public static class WarehouseSpectatorCameraVerification
{
    private static string OutputDirectory => Path.GetFullPath(
        Path.Combine(Application.dataPath, "../../ROS1/build/spectator-views"));
    private static double nextRequestCheck;
    private static PreviewRun previewRun;

    [Serializable]
    private class PreviewRequest
    {
        public float cameraX = 34f;
        public float cameraHeight = 6.5f;
        public float forwardOffset = 8f;
        public float maximumZ = 50f;
    }

    // A diagnostic request allows repeated visual checks in the open editor
    // without reloading scripts, entering Play, or modifying the saved scene.
    private static void CheckPreviewRequest()
    {
        if (EditorApplication.timeSinceStartup < nextRequestCheck ||
            EditorApplication.isPlayingOrWillChangePlaymode || previewRun != null)
            return;
        nextRequestCheck = EditorApplication.timeSinceStartup + 1;
        string path = Path.Combine(OutputDirectory, "request.json");
        if (!File.Exists(path)) return;
        try
        {
            var request = JsonUtility.FromJson<PreviewRequest>(File.ReadAllText(path));
            File.Delete(path);
            Verify(request);
        }
        catch (Exception error) { Debug.LogException(error); }
    }

    [InitializeOnLoadMethod]
    private static void CheckAfterCompilation()
    {
        EditorApplication.update += CheckPreviewRequest;
        EditorApplication.delayCall += () =>
        {
            // Rendering six diagnostic previews starts HDRP shader compilation
            // during the demo's import peak. Keep this check available on demand.
            if (Environment.GetCommandLineArgs().Contains(
                    "DeliveryDemoSceneLauncher.PrepareMainWarehouseForCameraDemo")) return;
            if (!EditorApplication.isPlayingOrWillChangePlaymode &&
                SceneManager.GetActiveScene().name == "MainWarehouse")
                Verify();
        };
    }

    [MenuItem("Drone Delivery/Verify Warehouse Spectator Clearance")]
    public static void Verify()
    {
        Verify(null);
    }

    private static void Verify(PreviewRequest request)
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode ||
            SceneManager.GetActiveScene().name != "MainWarehouse")
            throw new InvalidOperationException("Open MainWarehouse and stop Play before checking camera clearance.");
        var drone = UnityEngine.Object.FindFirstObjectByType<PackageHandling>();
        if (drone == null)
            throw new InvalidOperationException("Warehouse drone not found.");

        Vector3[] targets = {
            new Vector3(34, 2, 10), new Vector3(34, 6, 10),
            new Vector3(34, 6, 25), new Vector3(31, 6, 40),
            new Vector3(28, 6, 55), new Vector3(28, 1.4f, 55)
        };
        Physics.SyncTransforms();
        bool clear = true;
        int checkedPositions = 0;
        // Include intermediate ascent, travel and descent positions, not just
        // the snapshots. Unity Y is vertical throughout this editor check.
        for (int segment = 0; segment < targets.Length - 1; segment++)
        for (int step = 0; step <= 20; step++)
        {
            Vector3 target = Vector3.Lerp(targets[segment], targets[segment + 1], step / 20f);
            checkedPositions++;
            Vector3 camera = PreviewPosition(target, request);
            Vector3 offset = camera - target;
            // Match the deoccluder's target exclusion and camera radius.
            const float targetExclusion = 1.25f;
            var obstacles = Physics.SphereCastAll(target + offset.normalized * targetExclusion,
                    0.2f, offset.normalized, offset.magnitude - targetExclusion,
                    Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore)
                .Select(hit => hit.collider)
                .Concat(Physics.OverlapSphere(camera, 0.2f, Physics.DefaultRaycastLayers,
                    QueryTriggerInteraction.Ignore))
                .Where(collider => !collider.transform.IsChildOf(drone.transform) &&
                    collider.GetComponentInParent<DeliveryPackage>() == null)
                .Select(collider => collider.name).Distinct().ToArray();
            if (obstacles.Length != 0)
            {
                Debug.LogWarning($"SPECTATOR_CLEARANCE target={target} camera={camera} obstacles={string.Join(",", obstacles)}");
                clear = false;
            }
        }
        if (previewRun != null)
            throw new InvalidOperationException("A spectator preview is already running.");
        previewRun = new PreviewRun(drone.transform, targets, request);
        EditorApplication.update += previewRun.Tick;
        if (!clear)
        {
            Debug.LogWarning("Preferred spectator view intersects warehouse colliders; inspect rendered previews.");
            return;
        }
        Debug.Log($"WAREHOUSE_SPECTATOR_CLEARANCE_VERIFIED positions={checkedPositions}; collider check only, visual review required");
    }

    private static Vector3 PreviewPosition(Vector3 target, PreviewRequest request)
    {
        return request == null ? WarehouseSpectatorCamera.PreferredPosition(target) :
            new Vector3(request.cameraX, request.cameraHeight, Mathf.Min(target.z + request.forwardOffset, request.maximumZ));
    }

    private sealed class PreviewRun : IDisposable
    {
        private readonly Transform drone;
        private readonly Vector3[] poses;
        private readonly PreviewRequest request;
        private readonly GameObject cameraObject;
        private readonly Camera camera;
        private readonly RenderTexture target;
        private readonly Texture2D image;
        private readonly Vector3 originalPosition;
        private readonly Quaternion originalRotation;
        private int index;
        private int warmup;

        public PreviewRun(Transform drone, Vector3[] poses, PreviewRequest request)
        {
            CinemachineBrain brain = UnityEngine.Object.FindFirstObjectByType<CinemachineBrain>();
            if (brain == null || brain.OutputCamera == null)
                throw new InvalidOperationException("Spectator output camera not found.");
            this.drone = drone;
            this.poses = poses;
            this.request = request;
            originalPosition = drone.position;
            originalRotation = drone.rotation;
            Directory.CreateDirectory(OutputDirectory);
            cameraObject = new GameObject("Spectator Preview") { hideFlags = HideFlags.HideAndDontSave };
            camera = cameraObject.AddComponent<Camera>();
            camera.CopyFrom(brain.OutputCamera);
            var virtualCamera = UnityEngine.Object.FindObjectsByType<CinemachineCamera>(FindObjectsSortMode.None)
                .FirstOrDefault(candidate => candidate.Follow == drone);
            if (virtualCamera != null) camera.fieldOfView = virtualCamera.Lens.FieldOfView;
            camera.enabled = false;
            camera.aspect = 16f / 9f;
            var hd = cameraObject.AddComponent<HDAdditionalCameraData>();
            EditorUtility.CopySerialized(brain.OutputCamera.GetComponent<HDAdditionalCameraData>(), hd);
            target = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32);
            image = new Texture2D(1280, 720, TextureFormat.RGB24, false);
            target.Create();
            camera.targetTexture = target;
            SetPose();
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            AssemblyReloadEvents.beforeAssemblyReload += Dispose;
        }

        private void SetPose()
        {
            drone.position = poses[index];
            Vector3 position = PreviewPosition(drone.position, request);
            camera.transform.SetPositionAndRotation(position,
                WarehouseSpectatorCamera.ViewRotation(drone.position, position));
            Physics.SyncTransforms();
            warmup = 3;
            EditorApplication.QueuePlayerLoopUpdate();
        }

        public void Tick()
        {
            try
            {
                // GPU Resident Drawer updates transforms in the player loop.
                // Give each pose frames to reach the renderer before capturing.
                if (warmup-- > 0)
                {
                    EditorApplication.QueuePlayerLoopUpdate();
                    return;
                }
                RenderTexture previous = RenderTexture.active;
                try
                {
                    RenderPipeline.SubmitRenderRequest(camera,
                        new RenderPipeline.StandardRequest { destination = target });
                    RenderTexture.active = target;
                    image.ReadPixels(new Rect(0, 0, 1280, 720), 0, 0);
                    image.Apply();
                    File.WriteAllBytes(Path.Combine(OutputDirectory, $"view-{index}.png"), image.EncodeToPNG());
                }
                finally { RenderTexture.active = previous; }
                if (++index < poses.Length) SetPose();
                else
                {
                    Debug.Log("WAREHOUSE_SPECTATOR_PREVIEWS " + OutputDirectory);
                    Dispose();
                }
            }
            catch (Exception error)
            {
                Dispose();
                Debug.LogException(error);
            }
        }

        private void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingEditMode) Dispose();
        }

        public void Dispose()
        {
            EditorApplication.update -= Tick;
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            AssemblyReloadEvents.beforeAssemblyReload -= Dispose;
            if (drone != null) drone.SetPositionAndRotation(originalPosition, originalRotation);
            target.Release();
            UnityEngine.Object.DestroyImmediate(image);
            UnityEngine.Object.DestroyImmediate(target);
            UnityEngine.Object.DestroyImmediate(cameraObject);
            previewRun = null;
            EditorApplication.QueuePlayerLoopUpdate();
        }
    }
}
