using DroneSim;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace DroneSimEditor
{
    public static class DroneSimulationSceneBuilder
    {
        [MenuItem("Drone Sim/Create Starter Scene")]
        public static void CreateSceneFromMenu()
        {
            CreateStarterScene();
        }

        public static void CreateStarterScene()
        {
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            scene.name = "DroneSimulation";

            DroneHud hud = CreateHud();
            DroneController drone = CreateDrone();
            hud.SetDrone(drone);

            CreateEnvironment();
            CreateGates(hud);
            CreateLighting();
            CreateCamera(drone.transform);

            EditorSceneManager.SaveScene(scene, "Assets/Scenes/DroneSimulation.unity");
            if (!Application.isBatchMode)
            {
                EditorUtility.DisplayDialog("Drone Simulation", "Created Assets/Scenes/DroneSimulation.unity", "OK");
            }
        }

        private static DroneController CreateDrone()
        {
            GameObject root = new GameObject("Training Drone");
            root.transform.position = new Vector3(0f, 3f, 0f);

            Rigidbody body = root.AddComponent<Rigidbody>();
            body.useGravity = true;

            BoxCollider collider = root.AddComponent<BoxCollider>();
            collider.size = new Vector3(1.6f, 0.28f, 1.6f);

            DroneController controller = root.AddComponent<DroneController>();

            Material bodyMaterial = CreateMaterial("Drone Body", new Color(0.08f, 0.1f, 0.13f));
            Material accentMaterial = CreateMaterial("Drone Accent", new Color(0f, 0.75f, 1f));

            GameObject bodyVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            bodyVisual.name = "Body";
            bodyVisual.transform.SetParent(root.transform, false);
            bodyVisual.transform.localScale = new Vector3(0.9f, 0.18f, 0.9f);
            bodyVisual.GetComponent<Renderer>().material = bodyMaterial;
            Object.DestroyImmediate(bodyVisual.GetComponent<Collider>());

            Vector3[] armPositions =
            {
                new Vector3(0.72f, 0f, 0.72f),
                new Vector3(-0.72f, 0f, 0.72f),
                new Vector3(0.72f, 0f, -0.72f),
                new Vector3(-0.72f, 0f, -0.72f)
            };

            foreach (Vector3 localPosition in armPositions)
            {
                GameObject arm = GameObject.CreatePrimitive(PrimitiveType.Cube);
                arm.name = "Arm";
                arm.transform.SetParent(root.transform, false);
                arm.transform.localPosition = localPosition * 0.5f;
                arm.transform.localRotation = Quaternion.Euler(0f, localPosition.x * localPosition.z > 0f ? 45f : -45f, 0f);
                arm.transform.localScale = new Vector3(1.4f, 0.08f, 0.08f);
                arm.GetComponent<Renderer>().material = bodyMaterial;
                Object.DestroyImmediate(arm.GetComponent<Collider>());

                GameObject rotor = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                rotor.name = "Rotor";
                rotor.transform.SetParent(root.transform, false);
                rotor.transform.localPosition = localPosition;
                rotor.transform.localScale = new Vector3(0.36f, 0.025f, 0.36f);
                rotor.GetComponent<Renderer>().material = accentMaterial;
                rotor.AddComponent<SpinRotors>();
                Object.DestroyImmediate(rotor.GetComponent<Collider>());
            }

            return controller;
        }

        private static void CreateEnvironment()
        {
            Material groundMaterial = CreateMaterial("Mat Ground", new Color(0.18f, 0.25f, 0.2f));

            GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "Training Field";
            ground.transform.position = new Vector3(0f, -0.05f, 22f);
            ground.transform.localScale = new Vector3(80f, 0.1f, 80f);
            ground.GetComponent<Renderer>().material = groundMaterial;

            for (int i = -4; i <= 4; i++)
            {
                CreateMarker(new Vector3(i * 5f, 0.01f, 22f), new Vector3(0.08f, 0.02f, 80f));
                CreateMarker(new Vector3(0f, 0.02f, 22f + i * 5f), new Vector3(80f, 0.02f, 0.08f));
            }
        }

        private static void CreateMarker(Vector3 position, Vector3 scale)
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            marker.name = "Field Marker";
            marker.transform.position = position;
            marker.transform.localScale = scale;
            marker.GetComponent<Renderer>().material = CreateMaterial("Mat Field Marker", new Color(0.9f, 0.9f, 0.78f));
            Object.DestroyImmediate(marker.GetComponent<Collider>());
        }

        private static void CreateGates(DroneHud hud)
        {
            Vector3[] positions =
            {
                new Vector3(0f, 4f, 12f),
                new Vector3(5f, 5f, 24f),
                new Vector3(-6f, 6f, 38f),
                new Vector3(0f, 5f, 52f)
            };

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject gate = new GameObject($"Gate {i + 1}");
                gate.name = $"Gate {i + 1}";
                gate.transform.position = positions[i];

                BoxCollider collider = gate.AddComponent<BoxCollider>();
                collider.size = new Vector3(5.2f, 5.2f, 0.8f);
                collider.isTrigger = true;

                DroneGate droneGate = gate.AddComponent<DroneGate>();
                droneGate.SetHud(hud);
                Material material = CreateMaterial($"Gate {i + 1} Material", new Color(0f, 0.9f, 1f, 0.45f));
                CreateGateBar(gate.transform, "Top", new Vector3(0f, 2.6f, 0f), new Vector3(5.4f, 0.16f, 0.16f), material);
                CreateGateBar(gate.transform, "Bottom", new Vector3(0f, -2.6f, 0f), new Vector3(5.4f, 0.16f, 0.16f), material);
                CreateGateBar(gate.transform, "Left", new Vector3(-2.6f, 0f, 0f), new Vector3(0.16f, 5.4f, 0.16f), material);
                CreateGateBar(gate.transform, "Right", new Vector3(2.6f, 0f, 0f), new Vector3(0.16f, 5.4f, 0.16f), material);
            }
        }

        private static void CreateGateBar(Transform parent, string name, Vector3 localPosition, Vector3 localScale, Material material)
        {
            GameObject bar = GameObject.CreatePrimitive(PrimitiveType.Cube);
            bar.name = name;
            bar.transform.SetParent(parent, false);
            bar.transform.localPosition = localPosition;
            bar.transform.localScale = localScale;
            bar.GetComponent<Renderer>().material = material;
            Object.DestroyImmediate(bar.GetComponent<Collider>());
        }

        private static DroneHud CreateHud()
        {
            GameObject canvasObject = new GameObject("HUD", typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            Canvas canvas = canvasObject.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;

            CanvasScaler scaler = canvasObject.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);

            GameObject textObject = new GameObject("Readout", typeof(RectTransform), typeof(Text));
            textObject.transform.SetParent(canvasObject.transform, false);

            RectTransform rect = textObject.GetComponent<RectTransform>();
            rect.anchorMin = new Vector2(0f, 1f);
            rect.anchorMax = new Vector2(0f, 1f);
            rect.pivot = new Vector2(0f, 1f);
            rect.anchoredPosition = new Vector2(24f, -24f);
            rect.sizeDelta = new Vector2(360f, 180f);

            Text text = textObject.GetComponent<Text>();
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = 28;
            text.color = Color.white;
            text.alignment = TextAnchor.UpperLeft;

            DroneHud hud = canvasObject.AddComponent<DroneHud>();
            hud.SetReadout(text);
            return hud;
        }

        private static void CreateLighting()
        {
            GameObject sun = new GameObject("Sun", typeof(Light));
            sun.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            Light light = sun.GetComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.15f;

            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.32f, 0.35f, 0.4f);
        }

        private static void CreateCamera(Transform target)
        {
            GameObject cameraObject = new GameObject("Main Camera", typeof(Camera), typeof(AudioListener), typeof(DroneCameraFollow));
            cameraObject.tag = "MainCamera";
            cameraObject.transform.position = new Vector3(0f, 6f, -10f);

            DroneCameraFollow follow = cameraObject.GetComponent<DroneCameraFollow>();
            follow.SetTarget(target);
        }

        private static Material CreateMaterial(string name, Color color)
        {
            Material material = new Material(Shader.Find("Standard"));
            material.name = name;
            material.color = color;
            return material;
        }
    }
}
