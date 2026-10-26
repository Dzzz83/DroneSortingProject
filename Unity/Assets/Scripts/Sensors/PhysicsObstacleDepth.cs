using System;
using DroneDelivery;
using UnityEngine;

/// <summary>Sparse calibrated range measurements of the simulated solid surfaces.</summary>
public static class PhysicsObstacleDepth
{
    // Unmeasured pixels stay NaN. ROS samples this grid without interpolation.
    public const int PixelStride = 8;

    public static void Measure(RgbDepthFrame frame, Camera camera, Transform drone)
    {
        var ownColliders = drone.GetComponentsInChildren<Collider>();
        var enabled = new bool[ownColliders.Length];
        try
        {
            // Ignore only the drone itself, preserving attached-package occlusion.
            // No physics step occurs during this synchronous acquisition.
            for (int i = 0; i < ownColliders.Length; i++)
            {
                enabled[i] = ownColliders[i].enabled;
                if (ownColliders[i].GetComponentInParent<DeliveryPackage>() == null)
                    ownColliders[i].enabled = false;
            }
            Physics.SyncTransforms();
            Array.Fill(frame.DepthMetres, float.NaN);
            float fy = frame.Height / (2f * Mathf.Tan(camera.fieldOfView * Mathf.Deg2Rad * .5f));
            float fx = fy * frame.Width / (frame.Height * camera.aspect);
            float cx = (frame.Width - 1) * .5f, cy = (frame.Height - 1) * .5f;
            for (int row = 0; row < frame.Height; row += PixelStride)
            for (int column = 0; column < frame.Width; column += PixelStride)
            {
                // Optical coordinates: X right, Y down, Z forward. Physics uses
                // ray length; the ROS depth image contains optical-axis distance.
                var optical = new Vector3((column-cx)/fx, -(row-cy)/fy, 1f);
                float scale = optical.magnitude;
                Vector3 direction = frame.CameraRotation * (optical / scale);
                float depth = float.PositiveInfinity;
                if (Physics.Raycast(frame.CameraPosition, direction, out RaycastHit hit,
                        ForwardObstacleCamera.MaximumDepth * scale, Physics.DefaultRaycastLayers,
                        QueryTriggerInteraction.Ignore))
                {
                    depth = hit.distance / scale;
                    if (depth < ForwardObstacleCamera.MinimumDepth) depth = float.NaN;
                }
                frame.DepthMetres[row * frame.Width + column] = depth;
            }
        }
        finally
        {
            for (int i = 0; i < ownColliders.Length; i++)
                if (ownColliders[i] != null) ownColliders[i].enabled = enabled[i];
            Physics.SyncTransforms();
        }
    }
}
