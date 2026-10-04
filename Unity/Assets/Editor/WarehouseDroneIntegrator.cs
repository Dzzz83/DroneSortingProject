using System;
using System.IO;
using System.Reflection;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class WarehouseDroneIntegrator
{
    private const string ScenePath = "Assets/Scenes/MainWarehouse.unity";
    private const string DronePrefabPath = "Assets/Prefabs/Drone.prefab";
    private const string PackagePrefabPath = "Assets/Prefabs/Package.prefab";
    private const string IntegrationRootName = "Member2_DroneSystem";
    private const string DeliveryTargetName = "SortingZoneA";

    public static void Run()
    {
        ConvertDroneMaterialsToHdrp();

        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
            throw new InvalidOperationException("Could not open " + ScenePath);

        if (GameObject.Find(IntegrationRootName) != null)
            throw new InvalidOperationException(IntegrationRootName + " already exists. Integration was not applied twice.");

        var droneStart = RequireObject("DroneStart").transform;
        var pickupPoint = RequireObject("PickupPoint").transform;
        var pickupArea = RequireObject("PickupArea");
        var integrationPoints = GameObject.Find("IntegrationPoints");

        var dronePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(DronePrefabPath);
        var packagePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(PackagePrefabPath);
        if (dronePrefab == null || packagePrefab == null)
            throw new InvalidOperationException("Drone or package prefab is missing after asset copy.");

        var systemRoot = new GameObject(IntegrationRootName);
        if (integrationPoints != null)
            systemRoot.transform.SetParent(integrationPoints.transform, false);

        var drone = (GameObject)PrefabUtility.InstantiatePrefab(dronePrefab, scene);
        drone.name = "DeliveryDrone_ATLAS01";
        drone.transform.SetParent(systemRoot.transform, true);
        drone.transform.position = droneStart.position;

        Vector3 directionToPickup = pickupPoint.position - droneStart.position;
        directionToPickup.y = 0f;
        if (directionToPickup.sqrMagnitude > 0.001f)
        {
            // The visual sensors on this model face local -Z.
            drone.transform.rotation = Quaternion.LookRotation(-directionToPickup.normalized, Vector3.up);
        }

        var package = (GameObject)PrefabUtility.InstantiatePrefab(packagePrefab, scene);
        package.name = "DeliveryPackage_Pickup";
        package.transform.SetParent(systemRoot.transform, true);

        float surfaceY = pickupArea.transform.position.y;
        var surfaceRenderer = pickupArea.GetComponent<Renderer>();
        if (surfaceRenderer != null)
            surfaceY = surfaceRenderer.bounds.max.y;

        package.transform.position = new Vector3(pickupPoint.position.x, surfaceY + 0.31f, pickupPoint.position.z);
        package.transform.rotation = Quaternion.identity;

        var handling = drone.GetComponent<PackageHandling>();
        var deliveryPackage = package.GetComponent<DeliveryPackage>();
        var deliveryTarget = RequireObject(DeliveryTargetName).transform;
        if (handling == null || deliveryPackage == null)
            throw new InvalidOperationException("Drone or package is missing its delivery runtime component.");

        var demo = systemRoot.AddComponent<DeliveryDemo>();
        demo.drone = handling;
        demo.package = deliveryPackage;
        demo.delivery = new Vector3(deliveryTarget.position.x, package.transform.position.y, deliveryTarget.position.z);
        demo.speed = 4f;

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();

        string reportPath = "Assets/DroneDelivery/WarehouseIntegration.txt";
        File.WriteAllText(reportPath,
            "ATLAS 01 warehouse integration\n" +
            "Scene: " + ScenePath + "\n" +
            "Drone position: " + Format(drone.transform.position) + "\n" +
            "Package position: " + Format(package.transform.position) + "\n" +
            "Drone faces PickupPoint using local -Z.\n" +
            "Automatic DeliveryDemo target: " + DeliveryTargetName + " at " + Format(demo.delivery) + "\n");
        AssetDatabase.ImportAsset(reportPath);

        Debug.Log("WAREHOUSE_DRONE_INTEGRATION_COMPLETE drone=" + Format(drone.transform.position) +
                  " package=" + Format(package.transform.position));
    }

    private static GameObject RequireObject(string name)
    {
        var gameObject = GameObject.Find(name);
        if (gameObject == null)
            throw new InvalidOperationException("Required warehouse marker not found: " + name);
        return gameObject;
    }

    private static void ConvertDroneMaterialsToHdrp()
    {
        Shader hdrpLit = Shader.Find("HDRP/Lit");
        if (hdrpLit == null)
            throw new InvalidOperationException("HDRP/Lit shader is unavailable. Confirm that HDRP is active.");

        string[] materialGuids = AssetDatabase.FindAssets("t:Material", new[] { "Assets/DroneDelivery" });
        foreach (string guid in materialGuids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            Material source = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (source == null || source.shader == hdrpLit)
                continue;

            Color baseColor = source.HasProperty("_Color") ? source.GetColor("_Color") : source.color;
            Texture baseMap = source.HasProperty("_MainTex") ? source.GetTexture("_MainTex") : source.mainTexture;
            Vector2 baseScale = source.mainTextureScale;
            Vector2 baseOffset = source.mainTextureOffset;
            Texture normalMap = source.HasProperty("_BumpMap") ? source.GetTexture("_BumpMap") : null;
            float normalScale = source.HasProperty("_BumpScale") ? source.GetFloat("_BumpScale") : 1f;
            float metallic = source.HasProperty("_Metallic") ? source.GetFloat("_Metallic") : 0f;
            float smoothness = source.HasProperty("_Glossiness") ? source.GetFloat("_Glossiness") : 0.5f;
            Color emission = source.HasProperty("_EmissionColor") ? source.GetColor("_EmissionColor") : Color.black;

            var converted = new Material(hdrpLit) { name = source.name };
            converted.SetColor("_BaseColor", baseColor);
            converted.SetTexture("_BaseColorMap", baseMap);
            converted.SetTextureScale("_BaseColorMap", baseScale);
            converted.SetTextureOffset("_BaseColorMap", baseOffset);
            converted.SetFloat("_Metallic", metallic);
            converted.SetFloat("_Smoothness", smoothness);

            if (normalMap != null)
            {
                converted.SetTexture("_NormalMap", normalMap);
                converted.SetFloat("_NormalScale", normalScale);
                converted.EnableKeyword("_NORMALMAP_TANGENT_SPACE");
            }

            if (emission.maxColorComponent > 0.001f)
            {
                converted.SetColor("_EmissiveColor", emission);
                converted.EnableKeyword("_EMISSIVE_COLOR_MAP");
            }

            EditorUtility.CopySerialized(converted, source);
            UnityEngine.Object.DestroyImmediate(converted);
            ValidateHdrpMaterial(source);
            EditorUtility.SetDirty(source);
        }
    }

    private static void ValidateHdrpMaterial(Material material)
    {
        Type hdMaterialType = Type.GetType(
            "UnityEditor.Rendering.HighDefinition.HDMaterial, Unity.RenderPipelines.HighDefinition.Editor");
        MethodInfo validateMethod = hdMaterialType?.GetMethod(
            "ValidateMaterial", BindingFlags.Public | BindingFlags.Static);
        validateMethod?.Invoke(null, new object[] { material });
    }

    private static string Format(Vector3 value)
    {
        return string.Format("({0:0.###}, {1:0.###}, {2:0.###})", value.x, value.y, value.z);
    }
}
