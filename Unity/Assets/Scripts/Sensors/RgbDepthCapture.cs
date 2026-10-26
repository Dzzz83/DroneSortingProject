using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

/// <summary>
/// On-demand HDRP sensor capture. Depth is optical-axis distance in metres, not
/// ray length. Opaque depth-writing geometry is supported; transparent surfaces
/// without a depth pass are not. No detection or package lookup occurs here.
/// </summary>
public sealed class RgbDepthCapture : IDisposable
{
    private readonly DroneRgbCamera sensor;
    private readonly RenderTexture rgbTarget;
    private readonly RenderTexture depthTarget;
    private readonly Texture2D rgbReadback;
    private readonly Texture2D depthReadback;
    private readonly Material depthMaterial;
    private readonly MetricDepthPass depthPass;
    private readonly float minimumDepth;
    private readonly float maximumDepth;
    private bool disposed;
    private readonly bool reportClearRange;

    public RgbDepthCapture(DroneRgbCamera sensor, float minimumDepth = 0.2f, float maximumDepth = 15f, bool reportClearRange = false)
    {
        if (sensor == null)
            throw new ArgumentNullException(nameof(sensor));
        if (float.IsNaN(minimumDepth) || float.IsNaN(maximumDepth) ||
            float.IsInfinity(minimumDepth) || float.IsInfinity(maximumDepth) ||
            minimumDepth <= 0f || maximumDepth <= minimumDepth)
            throw new ArgumentOutOfRangeException(nameof(minimumDepth), "Depth range must be finite and positive.");
        if (!SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.RFloat) ||
            !SystemInfo.SupportsTextureFormat(TextureFormat.RFloat))
            throw new NotSupportedException("The GPU must support floating-point depth readback.");

        Shader shader = Resources.Load<Shader>("DroneMetricDepth");
        if (shader == null || !shader.isSupported)
            throw new InvalidOperationException("The HDRP metric-depth shader is unavailable.");

        this.sensor = sensor;
        this.minimumDepth = minimumDepth;
        this.maximumDepth = maximumDepth;
        this.reportClearRange = reportClearRange;
        rgbTarget = new RenderTexture(sensor.ImageWidth, sensor.ImageHeight, 24, RenderTextureFormat.ARGB32);
        depthTarget = new RenderTexture(sensor.ImageWidth, sensor.ImageHeight, 0,
            RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear);
        rgbReadback = new Texture2D(sensor.ImageWidth, sensor.ImageHeight, TextureFormat.RGB24, false);
        depthReadback = new Texture2D(sensor.ImageWidth, sensor.ImageHeight, TextureFormat.RFloat, false, true);
        depthMaterial = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        depthPass = new MetricDepthPass(sensor.SensorCamera, depthTarget, depthMaterial);
        try
        {
            if (!rgbTarget.Create() || !depthTarget.Create())
                throw new InvalidOperationException("Could not allocate sensor render targets.");
            CustomPassVolume.RegisterGlobalCustomPass(CustomPassInjectionPoint.BeforeTransparent, depthPass);
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    public RgbDepthFrame Capture()
    {
        if (disposed)
            throw new ObjectDisposedException(nameof(RgbDepthCapture));
        if (sensor.ImageWidth != rgbTarget.width || sensor.ImageHeight != rgbTarget.height)
            throw new InvalidOperationException("Recreate capture resources after changing image dimensions.");

        Camera camera = sensor.SensorCamera;
        RenderTexture originalTarget = camera.targetTexture;
        RenderTexture originalActive = RenderTexture.active;
        double captureTime = Time.timeAsDouble;
        Vector3 cameraPosition = camera.transform.position;
        Quaternion cameraRotation = camera.transform.rotation;

        try
        {
            camera.targetTexture = rgbTarget;
            depthPass.DidExecute = false;
            RenderPipeline.SubmitRenderRequest(camera,
                new RenderPipeline.StandardRequest { destination = rgbTarget });
            if (!depthPass.DidExecute)
                throw new InvalidOperationException("HDRP did not execute the sensor depth pass. Check Custom Pass frame settings.");

            ReadTarget(rgbTarget, rgbReadback);
            ReadTarget(depthTarget, depthReadback);
            byte[] rgbBottomUp = rgbReadback.GetRawTextureData<byte>().ToArray();
            float[] depthBottomUp = depthReadback.GetRawTextureData<float>().ToArray();
            byte[] rgbTopDown = new byte[rgbBottomUp.Length];
            float[] depthTopDown = new float[depthBottomUp.Length];

            // Unity readback starts at the bottom-left; ROS images start at top-left.
            int width = rgbTarget.width;
            int height = rgbTarget.height;
            for (int row = 0; row < height; row++)
            {
                int sourceOffset = (height - 1 - row) * width;
                int outputOffset = row * width;
                Array.Copy(rgbBottomUp, sourceOffset * 3, rgbTopDown, outputOffset * 3, width * 3);
                for (int column = 0; column < width; column++)
                {
                    float depth = depthBottomUp[sourceOffset + column];
                    // The depth pass explicitly emits zero for rendered sky/no geometry.
                    // Only the opt-in forward stream exposes a valid no-hit as +infinity.
                    // NaN/negative/too-near measurements remain invalid; the pickup stream
                    // retains its original NaN convention for every out-of-range value.
                    bool clear = reportClearRange && (depth == 0f || (float.IsFinite(depth) && depth > maximumDepth));
                    depthTopDown[outputOffset + column] = clear ? float.PositiveInfinity :
                        depth >= minimumDepth && depth <= maximumDepth ? depth : float.NaN;
                }
            }

            return new RgbDepthFrame(width, height, rgbTopDown, depthTopDown,
                captureTime, cameraPosition, cameraRotation);
        }
        finally
        {
            camera.targetTexture = originalTarget;
            RenderTexture.active = originalActive;
        }
    }

    private static void ReadTarget(RenderTexture target, Texture2D destination)
    {
        RenderTexture.active = target;
        destination.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0);
        destination.Apply();
    }

    public void Dispose()
    {
        if (disposed)
            return;
        disposed = true;
        CustomPassVolume.UnregisterGlobalCustomPass(depthPass);
        rgbTarget.Release();
        depthTarget.Release();
        ReleaseObject(rgbTarget);
        ReleaseObject(depthTarget);
        ReleaseObject(rgbReadback);
        ReleaseObject(depthReadback);
        ReleaseObject(depthMaterial);
    }

    private static void ReleaseObject(UnityEngine.Object value)
    {
        if (Application.isPlaying)
            UnityEngine.Object.Destroy(value);
        else
            UnityEngine.Object.DestroyImmediate(value);
    }

    private sealed class MetricDepthPass : CustomPass
    {
        private readonly Camera sensorCamera;
        private readonly RenderTexture destination;
        private readonly Material material;
        public bool DidExecute;

        public MetricDepthPass(Camera camera, RenderTexture destination, Material material)
        {
            sensorCamera = camera;
            this.destination = destination;
            this.material = material;
            targetColorBuffer = TargetBuffer.None;
            targetDepthBuffer = TargetBuffer.None;
        }

        protected override void Execute(CustomPassContext context)
        {
            if (context.hdCamera.camera != sensorCamera)
                return;
            context.cmd.SetRenderTarget(destination);
            context.cmd.SetViewport(new Rect(0, 0, destination.width, destination.height));
            CoreUtils.DrawFullScreen(context.cmd, material);
            DidExecute = true;
        }
    }
}
