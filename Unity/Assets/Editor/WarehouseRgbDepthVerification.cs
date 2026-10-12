using System;
using System.Diagnostics;
using System.IO;
using DroneDelivery;
using UnityEditor;
using UnityEngine;

/// <summary>Checks depth units, row order, alignment, invalid data, and warehouse capture.</summary>
public static class WarehouseRgbDepthVerification
{
    [MenuItem("Drone Delivery/Verify Warehouse RGB Depth Capture")]
    public static void Verify()
    {
        Require(!EditorApplication.isPlayingOrWillChangePlaymode, "Stop Play Mode before verification.");
        string directory = Path.Combine(Path.GetTempPath(), "drone-rgb-depth-verification");
        Directory.CreateDirectory(directory);
        VerifyControlledGeometry(directory);
        double milliseconds = VerifyWarehouse(directory);
        File.WriteAllText(Path.Combine(directory, "report.txt"),
            "PASS: 2 m optical-axis depth at central and off-axis probe pixels.\n" +
            "PASS: RGB/depth top-down alignment, sky invalidation, min/max range filtering.\n" +
            "PASS: repeat capture, disposal, and observation/open/closed-gripper capture.\n" +
            "Mean synchronous observation capture time (5 frames): " + milliseconds.ToString("F1") + " ms.\n" +
            "Sensor limits are provisional; no physical RGB-depth hardware or ROS stream tested.\n");
        UnityEngine.Debug.Log("WAREHOUSE_RGB_DEPTH_VERIFIED " + directory);
    }

    private static void VerifyControlledGeometry(string directory)
    {
        var root = new GameObject("TemporaryRgbDepthProbe") { hideFlags = HideFlags.HideAndDontSave };
        Material red = null;
        Material blue = null;
        try
        {
            // Isolate from warehouse geometry without changing its renderers or layers.
            root.transform.position = new Vector3(10000f, 10000f, 10000f);
            DroneRgbCamera sensor = DroneRgbCamera.EnsureOnDrone(root);
            sensor.transform.localPosition = Vector3.zero;
            sensor.transform.localRotation = Quaternion.identity;
            red = CreateMaterial(Color.red);
            blue = CreateMaterial(Color.blue);
            GameObject upper = CreateProbe(root.transform, new Vector3(0f, 0.7f, 2.3f), red);
            GameObject lower = CreateProbe(root.transform, new Vector3(0f, -0.7f, 2.3f), blue);

            using (var capture = new RgbDepthCapture(sensor))
            {
                capture.Capture(); // Warm the camera's exposure/history before RGB checks.
                RgbDepthFrame frame = capture.Capture();
                Require(frame.Rgb.Length == frame.Width * frame.Height * 3, "Incorrect RGB byte count.");
                Require(frame.DepthMetres.Length == frame.Width * frame.Height, "Incorrect depth count.");
                int upperPixel = PixelIndex(sensor.SensorCamera, frame, upper.transform.position);
                int lowerPixel = PixelIndex(sensor.SensorCamera, frame, lower.transform.position);
                Require(upperPixel / frame.Width < lowerPixel / frame.Width, "Image rows are upside down.");
                Require(Mathf.Abs(frame.DepthMetres[upperPixel] - 2f) < 0.02f, "Upper probe is not 2 m deep.");
                Require(Mathf.Abs(frame.DepthMetres[lowerPixel] - 2f) < 0.02f, "Lower probe is not 2 m deep.");
                Vector3 offAxisPoint = upper.transform.position + new Vector3(0.3f, 0f, 0f);
                int offAxisPixel = PixelIndex(sensor.SensorCamera, frame, offAxisPoint);
                Require(Mathf.Abs(frame.DepthMetres[offAxisPixel] - 2f) < 0.02f,
                    "Depth must be optical-axis distance, not slanted ray length.");
                Require(frame.Rgb[upperPixel * 3] > frame.Rgb[upperPixel * 3 + 2], "Upper RGB/depth alignment failed.");
                Require(frame.Rgb[lowerPixel * 3 + 2] > frame.Rgb[lowerPixel * 3], "Lower RGB/depth alignment failed.");
                Require(float.IsNaN(frame.DepthMetres[0]), "Sky must have invalid depth.");
                SaveImages(frame, directory, "controlled-probes");
            }

            using (var capture = new RgbDepthCapture(sensor, 2.1f, 3f))
            {
                RgbDepthFrame frame = capture.Capture();
                Require(float.IsNaN(frame.DepthMetres[PixelIndex(sensor.SensorCamera, frame, upper.transform.position)]),
                    "Depth below the minimum must be invalid.");
            }
            using (var capture = new RgbDepthCapture(sensor, 0.2f, 1.9f))
            {
                RgbDepthFrame frame = capture.Capture();
                Require(float.IsNaN(frame.DepthMetres[PixelIndex(sensor.SensorCamera, frame, lower.transform.position)]),
                    "Depth above the maximum must be invalid.");
            }
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(root);
            if (red != null) UnityEngine.Object.DestroyImmediate(red);
            if (blue != null) UnityEngine.Object.DestroyImmediate(blue);
        }
    }

    private static double VerifyWarehouse(string directory)
    {
        PackageHandling handling = UnityEngine.Object.FindFirstObjectByType<PackageHandling>();
        Require(handling != null, "MainWarehouse must be open with its delivery drone.");
        DroneRgbCamera sensor = handling.GetComponentInChildren<DroneRgbCamera>();
        Require(sensor != null, "Install the RGB camera first.");
        DeliveryPackage package = UnityEngine.Object.FindFirstObjectByType<DeliveryPackage>();
        Require(package != null, "No package available for the gripper view test.");
        Vector3 originalPosition = handling.transform.position;
        Quaternion originalRotation = handling.transform.rotation;
        Vector3 left = handling.gripper.leftJaw.localPosition;
        Vector3 right = handling.gripper.rightJaw.localPosition;

        try
        {
            handling.transform.SetPositionAndRotation(new Vector3(34f, 6f, 25f), Quaternion.identity);
            using (var capture = new RgbDepthCapture(sensor))
            {
                capture.Capture();
                var timer = Stopwatch.StartNew();
                RgbDepthFrame frame = null;
                for (int i = 0; i < 5; i++) frame = capture.Capture();
                timer.Stop();
                SaveImages(frame, directory, "observation");

                handling.transform.position += package.transform.position - handling.packageMount.position;
                SetJawWidth(handling.gripper, handling.gripper.openWidth);
                SaveImages(capture.Capture(), directory, "pickup-open");
                SetJawWidth(handling.gripper, handling.gripper.closedWidth);
                SaveImages(capture.Capture(), directory, "pickup-closed");
                return timer.Elapsed.TotalMilliseconds / 5;
            }
        }
        finally
        {
            handling.transform.SetPositionAndRotation(originalPosition, originalRotation);
            handling.gripper.leftJaw.localPosition = left;
            handling.gripper.rightJaw.localPosition = right;
        }
    }

    private static Material CreateMaterial(Color color)
    {
        Shader shader = Shader.Find("HDRP/Unlit");
        Require(shader != null, "HDRP/Unlit unavailable.");
        var material = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        material.SetColor("_UnlitColor", color);
        return material;
    }

    private static GameObject CreateProbe(Transform parent, Vector3 position, Material material)
    {
        GameObject probe = GameObject.CreatePrimitive(PrimitiveType.Cube);
        probe.transform.SetParent(parent, false);
        probe.transform.localPosition = position;
        probe.transform.localScale = new Vector3(1f, 1f, 0.6f);
        probe.GetComponent<Renderer>().sharedMaterial = material;
        return probe;
    }

    private static int PixelIndex(Camera camera, RgbDepthFrame frame, Vector3 worldPosition)
    {
        // A disabled camera without a target can have a zero-sized viewport in
        // the editor. Project using its lens rather than that display viewport.
        Vector3 local = camera.transform.InverseTransformPoint(worldPosition);
        float halfHeight = local.z * Mathf.Tan(camera.fieldOfView * Mathf.Deg2Rad * 0.5f);
        float horizontal = 0.5f + local.x / (2f * halfHeight * camera.aspect);
        float vertical = 0.5f + local.y / (2f * halfHeight);
        int column = Mathf.Clamp((int)(horizontal * frame.Width), 0, frame.Width - 1);
        int row = Mathf.Clamp((int)((1f - vertical) * frame.Height), 0, frame.Height - 1);
        return row * frame.Width + column;
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

    private static void SaveImages(RgbDepthFrame frame, string directory, string name)
    {
        var rgb = new Texture2D(frame.Width, frame.Height, TextureFormat.RGB24, false);
        var depth = new Texture2D(frame.Width, frame.Height, TextureFormat.RGB24, false);
        try
        {
            byte[] rgbBottomUp = new byte[frame.Rgb.Length];
            Color[] depthBottomUp = new Color[frame.DepthMetres.Length];
            for (int row = 0; row < frame.Height; row++)
            {
                int source = row * frame.Width;
                int destination = (frame.Height - 1 - row) * frame.Width;
                Array.Copy(frame.Rgb, source * 3, rgbBottomUp, destination * 3, frame.Width * 3);
                for (int column = 0; column < frame.Width; column++)
                {
                    float metres = frame.DepthMetres[source + column];
                    float shade = float.IsNaN(metres) ? 0f : Mathf.Clamp01(metres / 15f);
                    depthBottomUp[destination + column] = new Color(shade, shade, shade);
                }
            }
            rgb.LoadRawTextureData(rgbBottomUp);
            rgb.Apply();
            depth.SetPixels(depthBottomUp);
            depth.Apply();
            File.WriteAllBytes(Path.Combine(directory, name + "-rgb.png"), rgb.EncodeToPNG());
            File.WriteAllBytes(Path.Combine(directory, name + "-depth.png"), depth.EncodeToPNG());
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(rgb);
            UnityEngine.Object.DestroyImmediate(depth);
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
