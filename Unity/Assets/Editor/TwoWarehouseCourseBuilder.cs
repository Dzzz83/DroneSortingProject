using System;
using System.IO;
using System.Linq;
using DroneDelivery;
using Unity.Cinemachine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public static class TwoWarehouseCourseBuilder
{
    public const string ScenePath = "Assets/Scenes/TwoWarehouseDelivery.unity";
    const string Prefabs = "Assets/UnityWarehouseSceneHDRP/Scene_Warehouse/Background/Prefabs/";
    static Material wall, asphalt, blue, yellow, cardboard;

    [MenuItem("Drone Delivery/Build Two Warehouse Course")]
    public static void Build()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play before building.");
        if (File.Exists(ScenePath)) throw new InvalidOperationException("Course already exists; open it instead of overwriting edits.");
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/MainWarehouse.unity");
        wall = Material("CourseWall", new Color(.32f,.4f,.45f));
        asphalt = Material("CourseAsphalt", new Color(.12f,.14f,.16f));
        blue = Material("CourseBlue", new Color(.02f,.38f,.65f));
        yellow = Material("CourseYellow", new Color(.95f,.65f,.04f));
        cardboard = Material("CourseCardboard", new Color(.38f,.21f,.08f));
        var drone = UnityEngine.Object.FindAnyObjectByType<PackageHandling>();
        var package = UnityEngine.Object.FindObjectsByType<DeliveryPackage>().First(p=>p.name == "Package");
        foreach (var extra in UnityEngine.Object.FindObjectsByType<DeliveryPackage>())
            if (extra != package) UnityEngine.Object.DestroyImmediate(extra.gameObject);
        foreach (string name in new[]{"Zones","IntegrationPoints","Outside","Skybox","Skybox_Refrection_Only","SceneIDMap","Building_Collider"})
        {
            var obj = GameObject.Find(name);
            if (obj != null) UnityEngine.Object.DestroyImmediate(obj);
        }
        var warehouseA = new GameObject("Warehouse A");
        foreach (var root in scene.GetRootGameObjects())
        {
            if (root == warehouseA || root == drone.gameObject || root == package.gameObject || root.name == "Player" ||
                root.name == "Global Volume" || root.name == "StaticLightingSky" || root.name == "Reflection Probe") continue;
            // Keep a single sun; warehouse lamps are duplicated with their building.
            foreach (Light light in root.GetComponentsInChildren<Light>())
                if (light.type == LightType.Directional) light.transform.SetParent(null, true);
            root.transform.SetParent(warehouseA.transform, true);
        }
        var building = GameObject.Find("Building");
        foreach (var renderer in building.GetComponentsInChildren<MeshRenderer>())
        {
            if (renderer.name.StartsWith("Bldg_Wall") || renderer.name.StartsWith("Shutter") ||
                new[]{"Bldg_Gap_001","Bldg_Door_002","Bldg_DoorSus_002","Bldg_Exit_001","Bldg_Pipe_003","Bldg_Screw_001","Bldg_Sus_001"}.Contains(renderer.name))
            { renderer.gameObject.SetActive(false); continue; }
            var filter = renderer.GetComponent<MeshFilter>();
            if (filter != null && renderer.GetComponent<Collider>() == null)
                renderer.gameObject.AddComponent<MeshCollider>().sharedMesh = filter.sharedMesh;
        }
        // The source has narrow 2.5 m shutters and one combined collision mesh.
        // Replace its perimeter with visible walls and a 6 m loading doorway.
        Box("West wall", new Vector3(5.9f,4.5f,34.75f), new Vector3(.3f,9,64.8f), wall, warehouseA.transform);
        Box("East wall", new Vector3(43.45f,4.5f,34.75f), new Vector3(.3f,9,64.8f), wall, warehouseA.transform);
        Box("Rear wall", new Vector3(24.67f,4.5f,2.4f), new Vector3(37.55f,9,.3f), wall, warehouseA.transform);
        Box("Loading wall left", new Vector3(18.45f,4.5f,67.1f), new Vector3(25.1f,9,.3f), wall, warehouseA.transform);
        Box("Loading wall right", new Vector3(40.225f,4.5f,67.1f), new Vector3(6.45f,9,.3f), wall, warehouseA.transform);
        Box("Loading lintel", new Vector3(34,6.75f,67.1f), new Vector3(6,4.5f,.3f), wall, warehouseA.transform);
        var warehouseB = UnityEngine.Object.Instantiate(warehouseA);
        warehouseB.name = "Warehouse B";
        warehouseB.transform.SetPositionAndRotation(new Vector3(68,0,174.5f), Quaternion.Euler(0,180,0));
        // Remove old lightmaps for moved geometry; both warehouses use the existing realtime HDRP lighting.
        foreach(var renderer in UnityEngine.Object.FindObjectsByType<Renderer>()) renderer.lightmapIndex = -1;
        foreach(var probe in UnityEngine.Object.FindObjectsByType<ReflectionProbe>()) probe.enabled = false;

        var yard = new GameObject("Static loading yard");
        Box("Yard asphalt", new Vector3(34,-.18f,87.2f), new Vector3(76,.3f,42), asphalt, yard.transform);
        for(int i=0;i<9;i++) Box("Yard lane marking", new Vector3(34,.001f,70+i*4), new Vector3(.12f,.01f,1.8f), yellow, yard.transform, false);
        var vehicle = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/UnityWarehouseSceneHDRP/Scene_Warehouse/Movable/Prefabs/Forklift.prefab");
        Obstacle(vehicle, "Parked forklift", new Vector3(34,0,80), new Vector3(1.6f,1.6f,1.6f), 90, yard.transform);
        for(int i=0;i<3;i++)
        {
            Obstacle(AssetDatabase.LoadAssetAtPath<GameObject>(Prefabs+"Pallet.prefab"), "Static pallet stack", new Vector3(25,i*.25f,96), new Vector3(2,2,2),0,yard.transform);
            for(int j=0;j<2;j++) Obstacle(AssetDatabase.LoadAssetAtPath<GameObject>(Prefabs+"Cardboard_A1.prefab"), "Palletised cargo", new Vector3(24.5f+j, .75f+i*.8f,96), new Vector3(1.5f,1.5f,1.5f),0,yard.transform);
        }
        Box("Tall box stack", new Vector3(34,1.8f,97), new Vector3(5,3.6f,3), cardboard, yard.transform);
        for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            Box("Box stack seam",new Vector3(31.5f+i*1.66f,.6f+j*1.2f,95.49f),new Vector3(.025f,1.18f,.02f),wall,yard.transform,false);

        var pickup = Box("Pickup inside A",new Vector3(34,.07f,55),new Vector3(5,.14f,5),yellow,null);
        var delivery = Box("Delivery inside B",new Vector3(34,.07f,119.5f),new Vector3(5,.14f,5),blue,null);
        var destination = new GameObject("Delivery observation volume B");
        destination.transform.position = new Vector3(34,.9f,119.5f);
        var volume = destination.AddComponent<BoxCollider>(); volume.size = new Vector3(5,1.6f,5); volume.isTrigger = true;
        drone.transform.SetPositionAndRotation(new Vector3(34,2,50),Quaternion.identity);
        package.transform.SetPositionAndRotation(new Vector3(34,.45f,55),Quaternion.identity);
        WarehouseSetup.ConfigureDrone(drone.gameObject);
        // Match the collision envelope to the visible rotors, body and gripper.
        var bodyBox = drone.GetComponent<BoxCollider>(); bodyBox.center = new Vector3(0,.07f,0); bodyBox.size = new Vector3(2.6f,.3f,2.3f);
        foreach(var leg in drone.GetComponentsInChildren<MeshRenderer>().Where(r=>r.name == "Landing leg"))
            leg.gameObject.AddComponent<BoxCollider>();
        var course = drone.gameObject.AddComponent<WarehouseCourse>();
        drone.gameObject.AddComponent<HardcodedWarehouseFlight>();
        course.drone = drone; course.package = package; course.deliveryVolume = volume;
        course.packageSupports = UnityEngine.Object.FindObjectsByType<Collider>().Where(c=>c.name == "Bldg_Floor_001" || c.gameObject == pickup || c.gameObject == delivery || c.name == "Yard asphalt").ToArray();
        foreach(var camera in UnityEngine.Object.FindObjectsByType<CinemachineCamera>()) camera.enabled = false;
        foreach(var spectator in UnityEngine.Object.FindObjectsByType<SpectatorCamera>()) spectator.enabled = false;
        foreach(var brain in UnityEngine.Object.FindObjectsByType<CinemachineBrain>()) brain.enabled = false;
        var main = Camera.main; main.farClipPlane=500; main.fieldOfView=55;
        var views = main.gameObject.AddComponent<WarehouseCourseViews>(); views.drone=drone.transform;
        views.overviewPosition = new Vector3(125,150,15); views.overviewTarget = new Vector3(34,0,87);
        views.yardPosition = new Vector3(78,39,58); views.yardTarget = new Vector3(34,0,88);
        views.roofs = new[]{warehouseA,warehouseB}.SelectMany(w=>w.GetComponentsInChildren<MeshRenderer>()).Where(r=>r.bounds.min.y > 6).Cast<Renderer>().ToArray();
        main.transform.SetPositionAndRotation(views.yardPosition,Quaternion.LookRotation(views.yardTarget-views.yardPosition));
        foreach(var roof in views.roofs) roof.enabled=false;
        Marker("DroneStart",drone.transform.position); Marker("PickupPoint",package.transform.position);
        Marker("WarehouseAExit",new Vector3(34,3,67.1f)); Marker("WarehouseBEntrance",new Vector3(34,3,107.4f));
        Marker("DeliveryPoint",new Vector3(34,.45f,119.5f));
        Label("WAREHOUSE A | PICKUP",new Vector3(28,5,66.8f),Quaternion.identity);
        Label("WAREHOUSE B | DELIVERY",new Vector3(40,5,107.7f),Quaternion.Euler(0,180,0));
        ConfigurePresentation();
        EditorSceneManager.SaveScene(scene,ScenePath);
        AssetDatabase.SaveAssets();
        Verify();
        Debug.Log("TWO_WAREHOUSE_COURSE_BUILT " + ScenePath);
    }

    [MenuItem("Drone Delivery/Install Hardcoded A to B Flight")]
    public static void InstallHardcodedFlight()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play first.");
        var scene = EditorSceneManager.OpenScene(ScenePath);
        var course = UnityEngine.Object.FindAnyObjectByType<WarehouseCourse>();
        if (course.GetComponent<HardcodedWarehouseFlight>() == null)
            course.gameObject.AddComponent<HardcodedWarehouseFlight>();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log("HARDCODED_FLIGHT_INSTALLED");
    }
    public static void ConfigurePresentation()
    {
        var volume=UnityEngine.Object.FindObjectsByType<Volume>().First(v=>v.isGlobal && v.sharedProfile != null);
        const string path="Assets/Settings/TwoWarehouseVolume.asset";
        var profile=AssetDatabase.LoadAssetAtPath<VolumeProfile>(path);
        if(profile==null)
        {
            profile=ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile,path);
            foreach(var component in volume.sharedProfile.components)
            {
                var copy=UnityEngine.Object.Instantiate(component);
                profile.components.Add(copy); AssetDatabase.AddObjectToAsset(copy,profile);
            }
        }
        if(!profile.TryGet<MotionBlur>(out var blur)) { blur=profile.Add<MotionBlur>(); AssetDatabase.AddObjectToAsset(blur,profile); }
        blur.active=true; blur.intensity.Override(0); EditorUtility.SetDirty(blur);
        if(!profile.TryGet<DepthOfField>(out var focus)) { focus=profile.Add<DepthOfField>(); AssetDatabase.AddObjectToAsset(focus,profile); }
        focus.active=true; focus.focusMode.Override(DepthOfFieldMode.Off); EditorUtility.SetDirty(focus);
        volume.sharedProfile=profile;
        EditorUtility.SetDirty(profile);
        var views=Camera.main.GetComponent<WarehouseCourseViews>();
        views.overviewPosition=new Vector3(105,110,25);
        Camera.main.GetComponent<HDAdditionalCameraData>().antialiasing=HDAdditionalCameraData.AntialiasingMode.None;
        var stack=GameObject.Find("Tall box stack");
        stack.GetComponent<Renderer>().sharedMaterial=Material("CourseCardboard",new Color(.38f,.21f,.08f));
    }
    static Material Material(string name, Color color)
    {
        string path="Assets/Materials/"+name+".mat";
        var material=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(material != null) return material;
        material=new Material(Shader.Find("HDRP/Lit")); material.SetColor("_BaseColor",color); material.SetFloat("_Smoothness",.2f);
        AssetDatabase.CreateAsset(material,path); return material;
    }
    static GameObject Box(string name, Vector3 center, Vector3 size, Material material, Transform parent, bool collision=true)
    {
        var obj=GameObject.CreatePrimitive(PrimitiveType.Cube); obj.name=name; obj.transform.SetParent(parent,true);
        obj.transform.position=center; obj.transform.localScale=size; obj.GetComponent<Renderer>().sharedMaterial=material;
        if(!collision) UnityEngine.Object.DestroyImmediate(obj.GetComponent<Collider>());
        obj.isStatic=true; return obj;
    }
    static void Obstacle(GameObject prefab,string name,Vector3 position,Vector3 scale,float yaw,Transform parent)
    {
        if(prefab == null) throw new InvalidOperationException("Missing obstacle prefab: "+name);
        var obj=(GameObject)PrefabUtility.InstantiatePrefab(prefab); obj.name=name; obj.transform.SetParent(parent,true);
        obj.transform.SetPositionAndRotation(position,Quaternion.Euler(0,yaw,0)); obj.transform.localScale=scale;
        foreach(var animator in obj.GetComponentsInChildren<Animator>()) animator.enabled=false;
        foreach(var body in obj.GetComponentsInChildren<Rigidbody>()) { body.isKinematic=true; body.useGravity=false; }
        foreach(var behaviour in obj.GetComponentsInChildren<MonoBehaviour>()) behaviour.enabled=false;
        Bounds bounds=obj.GetComponentsInChildren<Renderer>()[0].bounds;
        foreach(var renderer in obj.GetComponentsInChildren<Renderer>()) bounds.Encapsulate(renderer.bounds);
        // A stationary bounding box gives the planner a conservative solid obstacle.
        var collision=new GameObject(name+" collision"); collision.transform.SetParent(parent,true);
        collision.transform.position=bounds.center; collision.AddComponent<BoxCollider>().size=bounds.size;
        obj.isStatic=true; collision.isStatic=true;
    }
    static void Marker(string name,Vector3 position) { new GameObject(name).transform.position=position; }
    static void Label(string text,Vector3 position,Quaternion rotation)
    {
        var obj=new GameObject(text); obj.transform.SetPositionAndRotation(position,rotation);
        var label=obj.AddComponent<TMPro.TextMeshPro>(); label.text=text; label.fontSize=5; label.color=Color.white;
        label.alignment=TMPro.TextAlignmentOptions.Center; label.rectTransform.sizeDelta=new Vector2(16,2);
    }

    [MenuItem("Drone Delivery/Verify Two Warehouse Course")]
    public static void Verify()
    {
        var course=UnityEngine.Object.FindAnyObjectByType<WarehouseCourse>();
        if(course==null) throw new InvalidOperationException("Open TwoWarehouseDelivery first.");
        if(GameObject.Find("Warehouse A")==null || GameObject.Find("Warehouse B")==null) throw new InvalidOperationException("Both warehouses required.");
        Physics.SyncTransforms();
        Vector3[] route={new Vector3(34,3,55),new Vector3(34,3,71),new Vector3(25,3,71),new Vector3(25,3,87),new Vector3(43,3,87),new Vector3(43,3,103),new Vector3(34,3,103),new Vector3(34,3,119.5f)};
        var failures=new System.Collections.Generic.List<string>();
        for(int segment=0;segment<route.Length-1;segment++)
        {
            Vector3 delta=route[segment+1]-route[segment];
            foreach(var hit in Physics.BoxCastAll(route[segment]+new Vector3(0,-.3f,0),new Vector3(1.5f,1,1.4f),delta.normalized,Quaternion.identity,delta.magnitude,Physics.DefaultRaycastLayers,QueryTriggerInteraction.Ignore))
                if(!hit.transform.IsChildOf(course.drone.transform) && !hit.transform.IsChildOf(course.package.transform)) failures.Add(segment+":"+hit.collider.name);
        }
        string report="Loaded scene: "+SceneManager.GetActiveScene().path+"\n"+
            "Carried envelope: 3 x 2 x 2.8 m; sampled route is a fixture, never a flight controller.\n"+
            "Bypass sweep failures: "+string.Join(",",failures.Distinct())+"\n";
        File.WriteAllText("Library/course-verification.txt",report);
        if(failures.Count>0) throw new InvalidOperationException(report);
        Debug.Log("TWO_WAREHOUSE_CLEARANCE_PASS");
    }
}
