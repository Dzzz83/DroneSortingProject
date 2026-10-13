using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

/// <summary>Editor-only ground-truth export. Never used by operational perception.</summary>
public static class WarehousePackageDatasetPilot
{
    private const string ScenePath = "Assets/Scenes/MainWarehouse.unity";
    private static readonly Color[] IdColors = { Color.red, Color.green, Color.blue };

    [Serializable]
    private sealed class Box
    {
        public int packageIndex, visiblePixels, left, top, width, height;
    }

    [Serializable]
    private sealed class Observation
    {
        public string name;
        public string split, configurationId, view;
        public Vector3 dronePositionUnity;
        public Vector3[] packagePositionsUnity;
        public Vector3[] packageRotationsUnity;
        public bool[] packageActive;
        public Box[] boxes;
    }

    [Serializable]
    private sealed class Manifest
    {
        public string className = "package";
        public int classId = 0;
        public string labelMeaning = "Tight visible-pixel bounds; top-left image origin. Opaque geometry only.";
        public string purpose = "Pilot label verification only; not train/validation/test splits.";
        public string datasetKind = "pilot";
        public int seed = 20261012;
        public int imageWidth, imageHeight;
        public Observation[] observations;
    }

    [MenuItem("Drone Delivery/Export Package Dataset Pilot")]
    public static void Export()
    {
        ExportDataset(false);
    }

    [MenuItem("Drone Delivery/Export Package Training Trial")]
    public static void ExportTrainingTrial()
    {
        ExportDataset(true);
    }

    private static void ExportDataset(bool trainingTrial)
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play Mode before exporting the pilot.");
        for (int i = 0; i < SceneManager.sceneCount; i++)
            if (SceneManager.GetSceneAt(i).isDirty)
                throw new InvalidOperationException("Save existing scene edits before exporting the pilot.");

        SceneSetup[] originalSetup = EditorSceneManager.GetSceneManagerSetup();
        string root = Environment.GetEnvironmentVariable(trainingTrial ? "DRONE_DATASET_TRIAL_OUTPUT" : "DRONE_DATASET_PILOT_OUTPUT");
        if (string.IsNullOrEmpty(root))
            root = Path.Combine(Path.GetTempPath(), trainingTrial ? "drone-package-training-trial" : "drone-package-dataset-pilot",
                DateTime.UtcNow.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture));
        root = Path.GetFullPath(root);
        Require(!root.StartsWith(Path.GetFullPath(Application.dataPath) + Path.DirectorySeparatorChar,
            StringComparison.Ordinal) && root != Path.GetFullPath(Application.dataPath),
            "Export datasets outside Unity/Assets.");
        if (Directory.Exists(root) && Directory.EnumerateFileSystemEntries(root).Any())
            throw new InvalidOperationException("Choose an empty output directory: " + root);

        try
        {
            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            ExportScene(root, trainingTrial);
        }
        finally
        {
            // No scene is saved. Reload discards every temporary transform/active-state edit.
            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            if (originalSetup.Length > 0)
                EditorSceneManager.RestoreSceneManagerSetup(originalSetup);
        }
        File.WriteAllText(Path.Combine(root, "COMPLETE.txt"),
            (trainingTrial ? "200 training-trial" : "16 pilot") + " views exported; scene reloaded without saving.\n");
        Debug.Log((trainingTrial ? "WAREHOUSE_PACKAGE_TRAINING_TRIAL_EXPORTED " : "WAREHOUSE_PACKAGE_DATASET_PILOT_EXPORTED ") + root);
    }

    private static void ExportScene(string root, bool trainingTrial)
    {
        PackageHandling handling = UnityEngine.Object.FindAnyObjectByType<PackageHandling>();
        Require(handling != null, "Warehouse drone missing.");
        DroneRgbCamera sensor = handling.GetComponentInChildren<DroneRgbCamera>();
        Require(sensor != null, "Install the mounted RGB camera first.");
        DeliveryPackage[] packages = UnityEngine.Object.FindObjectsByType<DeliveryPackage>()
            .OrderBy(package => package.transform.position.x).ToArray();
        Require(packages.Length == 3, "The pilot expects the three-package demo scene.");
        Shader shader = Shader.Find("Hidden/DroneDataset/VisiblePackageId");
        Require(shader != null && shader.isSupported, "Package ID shader unavailable.");
        Camera camera = sensor.SensorCamera;
        var target = new RenderTexture(sensor.ImageWidth, sensor.ImageHeight, 24, RenderTextureFormat.ARGB32);
        var mask = new RenderTexture(sensor.ImageWidth, sensor.ImageHeight, 0,
            RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
        var rgb = new Texture2D(sensor.ImageWidth, sensor.ImageHeight, TextureFormat.RGB24, false);
        var ids = new Texture2D(sensor.ImageWidth, sensor.ImageHeight, TextureFormat.RGB24, false, true);
        var material = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        var pass = new PackageIdPass(camera, packages, mask, material);
        RenderTexture originalActive = RenderTexture.active;
        RenderTexture originalTarget = camera.targetTexture;
        GameObject blocker = null;
        Material blockerMaterial = null;
        var observations = new List<Observation>();
        foreach (string folder in new[] { "images", "labels", "masks", "overlays" })
            Directory.CreateDirectory(Path.Combine(root, folder));
        try
        {
            Require(target.Create() && mask.Create(), "Cannot create capture targets.");
            camera.targetTexture = target;
            CustomPassVolume.RegisterGlobalCustomPass(CustomPassInjectionPoint.BeforeTransparent, pass);
            blocker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            blocker.name = "TemporaryDatasetOccluder";
            blocker.hideFlags = HideFlags.HideAndDontSave;
            blockerMaterial = new Material(Shader.Find("HDRP/Lit")) { hideFlags = HideFlags.HideAndDontSave };
            blockerMaterial.SetColor("_BaseColor", new Color(0.25f, 0.25f, 0.25f));
            blocker.GetComponent<Renderer>().sharedMaterial = blockerMaterial;
            for (int index = 0; index < (trainingTrial ? 200 : 16); index++)
            {
                string split = trainingTrial ? TrialSplit(index / 2) : "";
                string configurationId = trainingTrial ? "configuration-" + (index / 2).ToString("D3") : "";
                string view = trainingTrial ? (index % 2 == 0 ? "observation" : "approach") : "";
                string name = trainingTrial ? ConfigureTrialView(index, handling, packages, blocker, camera) :
                    ConfigureView(index, handling, packages, blocker, camera);
                foreach (string folder in new[] { "images", "labels", "masks", "overlays" })
                    Directory.CreateDirectory(Path.Combine(root, folder, split));
                pass.DidExecute = false;
                for (int warmup = 0; warmup < 3; warmup++)
                    RenderPipeline.SubmitRenderRequest(camera, new RenderPipeline.StandardRequest { destination = target });
                Require(pass.DidExecute, "HDRP did not execute the package visibility pass.");
                ReadTarget(target, rgb);
                ReadTarget(mask, ids);
                Box[] boxes = FindBoxes(ids.GetPixels32(), ids.width, ids.height);
                if (!trainingTrial && index == 0) Require(boxes.Length == 3, "Baseline must show three labeled packages.");
                if (!trainingTrial && (index == 10 || index == 11 || index == 13))
                    Require(boxes.Length == 0, "Empty/fully occluded view must have no labels: " + name);
                if (!trainingTrial && index == 12) Require(boxes.Length == 1, "Partially occluded package must remain labeled.");
                if (!trainingTrial && index == 14)
                    Require(boxes.Length == 1 && boxes[0].left == 0, "Left-edge package must be clipped at the image boundary.");
                if (!trainingTrial && index == 15)
                    Require(boxes.Length == 1 && boxes[0].left + boxes[0].width == rgb.width,
                        "Right-edge package must be clipped at the image boundary.");
                File.WriteAllBytes(Path.Combine(root, "images", name + ".png"), rgb.EncodeToPNG());
                File.WriteAllBytes(Path.Combine(root, "masks", name + ".png"), ids.EncodeToPNG());
                File.WriteAllText(Path.Combine(root, "labels", name + ".txt"),
                    string.Join("\n", boxes.Select(box => FormatLabel(box, rgb.width, rgb.height))) +
                    (boxes.Length > 0 ? "\n" : ""));
                DrawBoxes(rgb, boxes);
                File.WriteAllBytes(Path.Combine(root, "overlays", name + ".png"), rgb.EncodeToPNG());
                observations.Add(new Observation { name = name, dronePositionUnity = handling.transform.position,
                    split = split, configurationId = configurationId, view = view,
                    packagePositionsUnity = packages.Select(package => package.transform.position).ToArray(),
                    packageRotationsUnity = packages.Select(package => package.transform.eulerAngles).ToArray(),
                    packageActive = packages.Select(package => package.gameObject.activeSelf).ToArray(), boxes = boxes });
                if (index % 20 == 0) Debug.Log("PACKAGE_DATASET_CAPTURE " + index);
            }
            File.WriteAllText(Path.Combine(root, "manifest.json"), JsonUtility.ToJson(new Manifest {
                datasetKind = trainingTrial ? "training-trial" : "pilot",
                purpose = trainingTrial ? "Initial RGB fine-tuning trial; split by scene configuration." : "Pilot label verification only.",
                imageWidth = rgb.width, imageHeight = rgb.height, observations = observations.ToArray() }, true));
            if (trainingTrial)
                File.WriteAllText(Path.Combine(root, "packages.yaml"),
                    "path: " + "\"" + root.Replace("\\", "/") + "\"\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: package\n");
        }
        finally
        {
            CustomPassVolume.UnregisterGlobalCustomPass(pass);
            camera.targetTexture = originalTarget;
            RenderTexture.active = originalActive;
            target.Release(); mask.Release();
            foreach (UnityEngine.Object value in new UnityEngine.Object[] { target, mask, rgb, ids, material, blocker, blockerMaterial })
                if (value != null) UnityEngine.Object.DestroyImmediate(value);
        }
    }

    private static string TrialSplit(int configuration)
    {
        return configuration < 70 ? "train" : configuration < 85 ? "val" : "test";
    }

    private static float Between(System.Random random, float minimum, float maximum)
    {
        return minimum + (float)random.NextDouble() * (maximum - minimum);
    }

    private static string ConfigureTrialView(int index, PackageHandling handling, DeliveryPackage[] packages,
        GameObject blocker, Camera camera)
    {
        int configuration = index / 2;
        // A configuration's two camera views stay in one split and reuse the same package poses.
        var random = new System.Random(20261012 + configuration * 7919);
        bool empty = configuration % 5 == 0;
        int packageCount = empty ? 0 : 1 + configuration % 3;
        var positions = new List<Vector3>();
        for (int p = 0; p < packages.Length; p++)
        {
            Vector3 position = Vector3.zero;
            bool separated = false;
            for (int attempt = 0; attempt < 100 && !separated; attempt++)
            {
                position = new Vector3(Between(random, 32.1f, 35.9f), 0.44f, Between(random, 23.1f, 26.9f));
                separated = positions.All(other => Vector3.Distance(other, position) >= 1f);
            }
            Require(separated, "Could not generate separated package placements.");
            positions.Add(position);
            packages[p].transform.SetPositionAndRotation(position, Quaternion.Euler(0f, Between(random, 0f, 360f), 0f));
            packages[p].gameObject.SetActive(p < packageCount);
        }
        bool observation = index % 2 == 0;
        Vector3 center = observation || empty ? new Vector3(34f, 0.44f, 25f) : positions[0];
        float height = observation ? Between(random, 5.6f, 6.8f) : Between(random, 2.5f, 3.5f);
        handling.transform.SetPositionAndRotation(new Vector3(center.x + Between(random, -0.35f, 0.35f), height,
            center.z + Between(random, -0.35f, 0.35f)), Quaternion.Euler(0f, Between(random, -12f, 12f), 0f));
        blocker.SetActive(!empty && configuration % 4 == 1);
        if (blocker.activeSelf)
        {
            blocker.transform.SetPositionAndRotation(Vector3.Lerp(camera.transform.position, positions[0], 0.5f),
                camera.transform.rotation);
            blocker.transform.localScale = new Vector3(Between(random, 0.06f, 0.14f), 1.5f, 0.1f);
        }
        return TrialSplit(configuration) + "/configuration-" + configuration.ToString("D3") +
            (empty ? "-empty" : "") + (observation ? "-observation" : "-approach");
    }

    private static string ConfigureView(int index, PackageHandling handling, DeliveryPackage[] packages,
        GameObject blocker, Camera camera)
    {
        handling.transform.SetPositionAndRotation(new Vector3(34f, 6f, 25f), Quaternion.identity);
        blocker.SetActive(false);
        for (int p = 0; p < packages.Length; p++)
        {
            packages[p].gameObject.SetActive(true);
            packages[p].transform.SetPositionAndRotation(new Vector3(32.8f + p * 1.15f, 0.44f, 24f + p * 0.8f),
                Quaternion.Euler(0f, index * 23f + p * 15f, 0f));
        }
        if (index < 10)
        {
            handling.transform.position += new Vector3((index % 3 - 1) * 0.4f, -(index % 4) * 0.7f, (index % 2) * 0.4f);
            if (index == 0) handling.transform.position = new Vector3(34f, 6f, 25f);
            for (int p = 0; p < packages.Length; p++)
            {
                packages[p].gameObject.SetActive(p < 1 + index % 3 || index == 0);
                packages[p].transform.position += new Vector3(Mathf.Sin(index + p) * 0.3f, 0f, Mathf.Cos(index + p) * 0.3f);
            }
            return index.ToString("D2") + "-placement";
        }
        for (int p = 0; p < packages.Length; p++) packages[p].gameObject.SetActive(false);
        if (index <= 11)
        {
            if (index == 11) handling.transform.position = new Vector3(34.4f, 4f, 25.3f);
            return index.ToString("D2") + "-empty";
        }
        packages[0].gameObject.SetActive(true);
        // Lower the camera for the boundary probes so the packages stay on the open pickup floor,
        // rather than placing the extreme-FOV references inside distant warehouse clutter.
        if (index >= 14) handling.transform.position = new Vector3(34f, 1.6f, 25f);
        // Place the package at a chosen image location by intersecting the ray with its center-height plane.
        float viewportX = index == 14 ? 0f : index == 15 ? 1f : 0.5f;
        Ray ray = camera.ViewportPointToRay(new Vector3(viewportX, 0.35f, 0f));
        var plane = new Plane(Vector3.up, new Vector3(0f, 0.44f, 0f));
        Require(plane.Raycast(ray, out float distance), "Reference ray must intersect package center-height plane.");
        packages[0].transform.SetPositionAndRotation(ray.GetPoint(distance), Quaternion.identity);
        if (index <= 13)
        {
            blocker.SetActive(true);
            blocker.transform.SetPositionAndRotation(Vector3.Lerp(camera.transform.position, packages[0].transform.position, 0.5f),
                camera.transform.rotation);
            blocker.transform.localScale = new Vector3(index == 12 ? 0.12f : 2f, 2f, 0.1f);
            return index.ToString("D2") + (index == 12 ? "-partial-occlusion" : "-full-occlusion");
        }
        return index.ToString("D2") + "-image-edge";
    }

    private static Box[] FindBoxes(Color32[] pixels, int width, int height)
    {
        var result = new List<Box>();
        for (int id = 0; id < IdColors.Length; id++)
        {
            int left = width, right = -1, bottom = height, top = -1, count = 0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    Color32 pixel = pixels[y * width + x];
                    bool matches = id == 0 ? pixel.r > 200 && pixel.g < 20 && pixel.b < 20 :
                        id == 1 ? pixel.g > 200 && pixel.r < 20 && pixel.b < 20 :
                        pixel.b > 200 && pixel.r < 20 && pixel.g < 20;
                    if (!matches) continue;
                    count++; left = Math.Min(left, x); right = Math.Max(right, x);
                    bottom = Math.Min(bottom, y); top = Math.Max(top, y);
                }
            if (count > 0)
                result.Add(new Box { packageIndex = id, visiblePixels = count, left = left,
                    top = height - 1 - top, width = right - left + 1, height = top - bottom + 1 });
        }
        return result.ToArray();
    }

    private static string FormatLabel(Box box, int width, int height)
    {
        return string.Format(CultureInfo.InvariantCulture, "0 {0:F8} {1:F8} {2:F8} {3:F8}",
            (box.left + box.width / 2f) / width, (box.top + box.height / 2f) / height,
            (float)box.width / width, (float)box.height / height);
    }

    private static void DrawBoxes(Texture2D image, Box[] boxes)
    {
        foreach (Box box in boxes)
        {
            int bottom = image.height - box.top - box.height;
            int top = image.height - 1 - box.top;
            Color color = IdColors[box.packageIndex];
            for (int x = box.left; x < box.left + box.width; x++)
            { image.SetPixel(x, bottom, color); image.SetPixel(x, top, color); }
            for (int y = bottom; y <= top; y++)
            { image.SetPixel(box.left, y, color); image.SetPixel(box.left + box.width - 1, y, color); }
        }
        image.Apply();
    }

    private static void ReadTarget(RenderTexture target, Texture2D destination)
    {
        RenderTexture.active = target;
        destination.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0);
        destination.Apply();
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private sealed class PackageIdPass : CustomPass
    {
        private readonly Camera camera;
        private readonly DeliveryPackage[] packages;
        private readonly RenderTexture destination;
        private readonly Material material;
        public bool DidExecute;

        public PackageIdPass(Camera camera, DeliveryPackage[] packages, RenderTexture destination, Material material)
        {
            this.camera = camera; this.packages = packages; this.destination = destination; this.material = material;
            targetColorBuffer = TargetBuffer.None;
            targetDepthBuffer = TargetBuffer.None;
        }

        protected override void Execute(CustomPassContext context)
        {
            if (context.hdCamera.camera != camera) return;
            context.cmd.SetRenderTarget(destination);
            context.cmd.SetViewport(new Rect(0, 0, destination.width, destination.height));
            context.cmd.ClearRenderTarget(false, true, Color.black);
            for (int id = 0; id < packages.Length; id++)
            {
                if (!packages[id].gameObject.activeInHierarchy) continue;
                context.cmd.SetGlobalColor("_DatasetPackageColor", IdColors[id]);
                foreach (Renderer renderer in packages[id].GetComponentsInChildren<Renderer>())
                    if (renderer.enabled)
                        for (int submesh = 0; submesh < renderer.sharedMaterials.Length; submesh++)
                            context.cmd.DrawRenderer(renderer, material, submesh, 0);
            }
            DidExecute = true;
        }
    }
}
