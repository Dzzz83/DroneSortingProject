using System;
using System.IO;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

/// <summary>Installs the sensor in MainWarehouse and verifies its actual views.</n+/// Package transforms are test references only, never perception inputs.</summary>
public static class WarehouseRgbCameraVerification
{
    private const string ScenePath = "Assets/Scenes/MainWarehouse.unity";

    [MenuItem("Drone Delivery/Install and Verify Warehouse RGB Camera")]
    public static void InstallAndVerify()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play Mode before installing the sensor.");

        Scene activeScene = SceneManager.GetActiveScene();
        if (activeScene.isDirty)
            throw new InvalidOperationException("Save or discard existing scene edits first.");

        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        PackageHandling handling = UnityEngine.Object.FindFirstObjectByType<PackageHandling>();
        Require(handling != null, "Warehouse drone not found.");

        DroneRgbCamera sensor = DroneRgbCamera.EnsureOnDrone(handling.gameObject);
        Require(DroneRgbCamera.EnsureOnDrone(handling.gameObject) == sensor,
            "Repeated configuration must reuse the sensor.");
        Require(handling.GetComponentsInChildren<DroneRgbCamera>().Length == 1,
            "Expected exactly one RGB sensor.");
        Require(Vector3.Distance(sensor.transform.localPosition, DroneRgbCamera.MountPosition) < 0.0001f,
            "Incorrect local mount position.");
        Require(Quaternion.Angle(sensor.transform.localRotation, DroneRgbCamera.MountRotation) < 0.001f,
            "Incorrect local mount rotation.");
        Require(!sensor.SensorCamera.enabled, "Sensor must not render to the spectator display.");
        Require(Mathf.Abs(sensor.SensorCamera.fieldOfView - 110f) < 0.001f,
            "Preview verification expects the provisional 110-degree vertical FOV.");

        string outputDirectory = Path.Combine(Path.GetTempPath(), "drone-permanent-rgb-verification");
        Directory.CreateDirectory(outputDirectory);
        VerifyViews(handling, sensor, outputDirectory);

        // Save only after verification has restored the original drone and jaws.
        EditorSceneManager.MarkSceneDirty(scene);
        Require(EditorSceneManager.SaveScene(scene), "Could not save the camera installation.");
        File.WriteAllText(Path.Combine(outputDirectory, "report.txt"),
            "PASS: one persistent RGB camera; setup is idempotent; mount/lens correct.\n" +
            "PASS: pickup-area corners and all three package bounds fit at observation.\n" +
            "PASS: target bounds fit at approach and with open/closed jaws at pickup.\n" +
            "Rendered views require visual review for occlusion. No depth or ROS capture tested.\n");
        Debug.Log("WAREHOUSE_RGB_CAMERA_VERIFIED " + outputDirectory);
    }

    private static void VerifyViews(PackageHandling handling, DroneRgbCamera sensor, string directory)
    {
        DeliveryPackage[] packages = UnityEngine.Object.FindObjectsByType<DeliveryPackage>(FindObjectsSortMode.None);
        Require(packages.Length == 3, "Expected the three-package demo scene.");
        DeliveryPackage target = packages[0];
        foreach (DeliveryPackage package in packages)
        {
            if (Vector3.Distance(package.transform.position, new Vector3(34.23f, 0.44f, 25.10f)) <
                Vector3.Distance(target.transform.position, new Vector3(34.23f, 0.44f, 25.10f)))
                target = package;
        }

        Transform drone = handling.transform;
        GripperController gripper = handling.gripper;
        Require(gripper != null, "Missing gripper.");
        Vector3 originalPosition = drone.position;
        Quaternion originalRotation = drone.rotation;
        Vector3 originalLeft = gripper.leftJaw.localPosition;
        Vector3 originalRight = gripper.rightJaw.localPosition;
        Camera camera = sensor.SensorCamera;
        RenderTexture originalTarget = camera.targetTexture;
        RenderTexture originalActive = RenderTexture.active;
        var renderTarget = new RenderTexture(sensor.ImageWidth, sensor.ImageHeight, 24, RenderTextureFormat.ARGB32);
        var image = new Texture2D(sensor.ImageWidth, sensor.ImageHeight, TextureFormat.RGB24, false);

        try
        {
            renderTarget.Create();
            camera.targetTexture = renderTarget;
            drone.rotation = Quaternion.identity;
            SetJawWidth(gripper, gripper.openWidth);
            drone.position = new Vector3(34f, 6f, 25f);
            foreach (DeliveryPackage package in packages)
                RequireBoundsVisible(camera, package.GetComponentInChildren<Renderer>().bounds);
            foreach (float x in new[] { 31.5f, 36.5f })
                foreach (float z in new[] { 22.5f, 27.5f })
                    RequirePointVisible(camera, new Vector3(x, 0.12f, z));
            SaveView(camera, renderTarget, image, directory, "observation");

            Vector3 packagePosition = target.transform.position;
            drone.position = new Vector3(packagePosition.x, 2.5f, packagePosition.z);
            RequireBoundsVisible(camera, target.GetComponentInChildren<Renderer>().bounds);
            SaveView(camera, renderTarget, image, directory, "approach");

            // Align the actual mount to the package, rather than assuming a fixed offset.
            drone.position += packagePosition - handling.packageMount.position;
            RequireBoundsVisible(camera, target.GetComponentInChildren<Renderer>().bounds);
            SaveView(camera, renderTarget, image, directory, "pickup-open");
            SetJawWidth(gripper, gripper.closedWidth);
            RequireBoundsVisible(camera, target.GetComponentInChildren<Renderer>().bounds);
            SaveView(camera, renderTarget, image, directory, "pickup-closed");
        }
        finally
        {
            drone.SetPositionAndRotation(originalPosition, originalRotation);
            gripper.leftJaw.localPosition = originalLeft;
            gripper.rightJaw.localPosition = originalRight;
            camera.targetTexture = originalTarget;
            RenderTexture.active = originalActive;
            renderTarget.Release();
            UnityEngine.Object.DestroyImmediate(renderTarget);
            UnityEngine.Object.DestroyImmediate(image);
        }
    }

    private static void SetJawWidth(GripperController gripper, float width)
    {
        Vector3 left = gripper.leftJaw.localPosition;
        Vector3 right = gripper.rightJaw.localPosition;
        left.x = -width;
        right.x = width;
        gripper.leftJaw.localPosition = left;
        gripper.rightJaw.localPosition = right;
    }

    private static void RequireBoundsVisible(Camera camera, Bounds bounds)
    {
        foreach (float x in new[] { bounds.min.x, bounds.max.x })
            foreach (float y in new[] { bounds.min.y, bounds.max.y })
                foreach (float z in new[] { bounds.min.z, bounds.max.z })
                    RequirePointVisible(camera, new Vector3(x, y, z));
    }

    private static void RequirePointVisible(Camera camera, Vector3 point)
    {
        Vector3 viewport = camera.WorldToViewportPoint(point);
        Require(viewport.z > camera.nearClipPlane && viewport.z < camera.farClipPlane &&
            viewport.x >= 0f && viewport.x <= 1f && viewport.y >= 0f && viewport.y <= 1f,
            "Reference point lies outside the sensor frustum: " + point);
    }

    private static void SaveView(Camera camera, RenderTexture target, Texture2D image,
        string directory, string name)
    {
        var request = new RenderPipeline.StandardRequest { destination = target };
        // Warm HDRP history before reading the reference image.
        for (int frame = 0; frame < 3; frame++)
            RenderPipeline.SubmitRenderRequest(camera, request);
        RenderTexture.active = target;
        image.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0);
        image.Apply();
        File.WriteAllBytes(Path.Combine(directory, name + ".png"), image.EncodeToPNG());
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}
