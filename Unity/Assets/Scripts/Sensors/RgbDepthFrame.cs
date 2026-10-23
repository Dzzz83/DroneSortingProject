using UnityEngine;

/// <summary>A co-located RGB/depth observation, with rows ordered top to bottom.</summary>
public sealed class RgbDepthFrame
{
    public readonly int Width;
    public readonly int Height;
    public readonly byte[] Rgb;
    public readonly float[] DepthMetres;
    public readonly double CaptureTime;
    public readonly Vector3 CameraPosition;
    public readonly Quaternion CameraRotation;

    public RgbDepthFrame(int width, int height, byte[] rgb, float[] depthMetres,
        double captureTime, Vector3 cameraPosition, Quaternion cameraRotation)
    {
        Width = width;
        Height = height;
        Rgb = rgb;
        DepthMetres = depthMetres;
        CaptureTime = captureTime;
        CameraPosition = cameraPosition;
        CameraRotation = cameraRotation;
    }
}
