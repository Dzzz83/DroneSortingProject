using System;
using System.Collections;
using System.IO;
using System.Linq;
using System.Reflection;
using DroneDelivery;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>Editor-only physical fixture. It is never part of a player or ROS navigation.</summary>
[InitializeOnLoad]
public static class TwoWarehouseCourseVerification
{
    const string Running = "TwoWarehouse.Verification";
    static IEnumerator run;
    static bool failed;
    static readonly System.Collections.Generic.List<string> checks = new System.Collections.Generic.List<string>();
    static TwoWarehouseCourseVerification() { EditorApplication.update += Tick; }

    [MenuItem("Drone Delivery/Run Two Warehouse Play Checks")]
    public static void Begin()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play first.");
        SessionState.SetBool("TwoWarehouse.FlightVerification", false);
        if (Application.isBatchMode) UnityEditor.SceneManagement.EditorSceneManager.OpenScene(TwoWarehouseCourseBuilder.ScenePath);
        TwoWarehouseCourseBuilder.Verify();
        var flight=UnityEngine.Object.FindAnyObjectByType<HardcodedWarehouseFlight>();
        if(flight == null || flight.enabled)
            throw new InvalidOperationException("The saved ROS course must have local flight disabled.");
        // The runtime initializer disables the network only for this editor fixture.
        SessionState.SetBool(Running,true);
        EditorApplication.isPlaying=true;
    }
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void PrepareConnection()
    {
        if(SessionState.GetBool(Running,false)) ROSConnection.GetOrCreateInstance().ConnectOnStart=false;
    }
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void PrepareFixture()
    {
        if (SessionState.GetBool(Running,false))
        {
            var flight=UnityEngine.Object.FindAnyObjectByType<HardcodedWarehouseFlight>();
            // Local flight is enabled only for its explicitly selected offline fixture.
            if(flight != null) flight.enabled=SessionState.GetBool("TwoWarehouse.FlightVerification",false);
        }
    }

    [MenuItem("Drone Delivery/Verify Course Spectator Controls")]
    public static void VerifyCourseSpectatorControls()
    {
        var cameraObject = new GameObject("Course spectator fixture");
        var target = new GameObject("Spectator target fixture");
        try
        {
            cameraObject.AddComponent<Camera>();
            var recordingCamera = cameraObject.AddComponent<UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData>();
            var views = cameraObject.AddComponent<WarehouseCourseViews>();
            // Also supports running this fixture with Play stopped.
            views.InitializeCamera();
            views.ConfigureRecordingRendering();
            foreach (var field in new[] {
                UnityEngine.Rendering.HighDefinition.FrameSettingsField.SSAO,
                UnityEngine.Rendering.HighDefinition.FrameSettingsField.SSGI,
                UnityEngine.Rendering.HighDefinition.FrameSettingsField.Volumetrics })
                Require(!recordingCamera.renderingPathCustomFrameSettings.IsEnabled(field) &&
                        recordingCamera.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)field],
                    "The recording spectator disables expensive effects through camera-local overrides.");
            views.drone = target.transform;
            target.transform.position = new Vector3(-100,10,-100);
            views.overviewPosition = new Vector3(-90,20,-110);
            views.overviewTarget = target.transform.position;
            views.yardPosition = new Vector3(-80,15,-100);
            views.yardTarget = target.transform.position;
            views.SelectView(1);
            Vector3 originalOffset = cameraObject.transform.position-target.transform.position;
            views.AdjustView(new Vector2(10,5),new Vector3(1,1,1),false,.1f);
            views.RefreshView();
            Vector3 customOffset = cameraObject.transform.position-target.transform.position;
            Require((customOffset-originalOffset).magnitude>.1f,"Manual follow controls change the spectator offset.");
            Vector3 displacement = new Vector3(2,1,80);
            target.transform.position += displacement;
            Vector3 beforeFollow = cameraObject.transform.position;
            views.RefreshView(1f/60f);
            Require(Vector3.Distance(cameraObject.transform.position,beforeFollow)>0 &&
                    Vector3.Distance(cameraObject.transform.position,beforeFollow)<displacement.magnitude,
                "Passive follow moves gradually rather than snapping to the drone.");
            for (int frame=0; frame<120; frame++) views.RefreshView(1f/60f);
            Require((cameraObject.transform.position-target.transform.position-customOffset).magnitude<.001f,
                "Adjusted follow offset persists while the drone moves beyond the original warehouse.");
            Require(Vector3.Dot(cameraObject.transform.forward,
                    (target.transform.position-cameraObject.transform.position).normalized)>.999f,
                "Adjusted follow view keeps looking at the drone.");
            views.SelectView(1);
            Require((cameraObject.transform.position-target.transform.position-originalOffset).magnitude<.001f,
                "Selecting follow resets its default offset.");
            views.SelectView(0);
            views.AdjustView(new Vector2(10,5),Vector3.forward,true,.1f);
            Vector3 freePosition = cameraObject.transform.position;
            Quaternion freeRotation = cameraObject.transform.rotation;
            target.transform.position += displacement;
            views.RefreshView();
            Require(cameraObject.transform.position==freePosition && cameraObject.transform.rotation==freeRotation,
                "Free camera stays where the user placed it.");
            views.SelectView(2);
            Require(cameraObject.transform.position==views.yardPosition,"Yard preset restores its fixed view.");
            Debug.Log("COURSE_SPECTATOR_CHECKS_PASS");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(cameraObject);
            UnityEngine.Object.DestroyImmediate(target);
        }
    }

    [MenuItem("Drone Delivery/Verify Obstacle Depth")]
    public static void VerifyObstacleDepth()
    {
        var root = new GameObject("Depth fixture drone");
        var cameraObject = new GameObject("Depth fixture camera");
        var obstacle = new GameObject("Invisible solid depth fixture");
        try
        {
            var camera = cameraObject.AddComponent<Camera>();
            camera.enabled = false; camera.fieldOfView = 90; camera.aspect = 1;
            camera.transform.position = new Vector3(-100,100,-100);
            camera.transform.rotation = Quaternion.identity;
            obstacle.transform.position = camera.transform.position + Vector3.forward*3;
            var box = obstacle.AddComponent<BoxCollider>(); box.size = new Vector3(5,5,.1f);
            Physics.SyncTransforms();
            var frame = new RgbDepthFrame(17,17,new byte[17*17*3],new float[17*17],
                0,camera.transform.position,camera.transform.rotation);
            PhysicsObstacleDepth.Measure(frame,camera,root.transform);
            Require(Mathf.Abs(frame.DepthMetres[8*17+8]-2.95f)<.001f,
                "Depth measures a collision-only surface at calibrated optical distance.");
            Require(float.IsNaN(frame.DepthMetres[1]),"Unsampled depth pixels remain unknown.");
            var stamp = DroneCameraMessages.UtcStamp();
            var depthOnly = DroneCameraMessages.BuildDepth(frame,camera,root.transform.position,
                Quaternion.identity,1,stamp,ForwardObstacleCamera.OpticalFrame);
            Require(depthOnly.Rgb==null && depthOnly.Depth.encoding=="32FC1" &&
                    depthOnly.Depth.header.stamp==depthOnly.Calibration.header.stamp &&
                    depthOnly.Depth.header.stamp==depthOnly.CameraPose.header.stamp &&
                    depthOnly.Depth.header.stamp==depthOnly.DronePose.header.stamp,
                "Physics depth has no fabricated RGB and retains matching acquisition stamps.");
            box.isTrigger = true;
            Physics.SyncTransforms();
            PhysicsObstacleDepth.Measure(frame,camera,root.transform);
            Require(float.IsPositiveInfinity(frame.DepthMetres[8*17+8]),
                "Triggers do not supply solid obstacle returns.");
            Debug.Log("OBSTACLE_DEPTH_CHECKS_PASS");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(obstacle);
            UnityEngine.Object.DestroyImmediate(cameraObject);
            UnityEngine.Object.DestroyImmediate(root);
        }
    }

    [Serializable]
    class TrajectoryFixturePacket
    {
        public int reset_sequence, sequence;
        public bool stop, restart;
        public long issued_ms = 100000, expires_ms = 100200;
        public float speed = 2;
        public Vector3 start, end;
    }

    // Deterministic checks exercise the real executor with an explicit clock.
    public static void VerifyNavigationSmoothness()
    {
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene(TwoWarehouseCourseBuilder.ScenePath);
        VerifyCourseSpectatorControls();
        VerifyObstacleDepth();
        var course = UnityEngine.Object.FindAnyObjectByType<WarehouseCourse>();
        var executor = course.GetComponent<DroneCommandSubscriber>();
        typeof(WarehouseCourse).GetMethod("Awake",BindingFlags.Instance|BindingFlags.NonPublic).Invoke(course,null);
        Vector3 saved = executor.transform.position;
        Vector3 origin = new Vector3(-100,100,-100);
        Vector3 Ros(Vector3 value) => new Vector3(value.x,value.z,value.y);
        void Packet(int sequence, bool restart, Vector3 start, Vector3 end, bool stop=false, int epochOffset=0)
        {
            executor.ApplyApprovedTrajectory(JsonUtility.ToJson(new TrajectoryFixturePacket {
                reset_sequence=course.ResetSequence+epochOffset, sequence=sequence,
                restart=restart, start=Ros(start), end=Ros(end), stop=stop }),100000);
        }
        try
        {
            executor.transform.position=origin;
            executor.ResetForTrial();
            Packet(1,false,origin,origin,true);
            string stopAcknowledgement = executor.TrajectoryStatusJson(100000);
            Packet(1,false,origin,origin,true);
            Require(executor.TrajectoryStatusJson(100000)==stopAcknowledgement,
                "Retried stop packets preserve the acknowledged stop identity and position.");
            Packet(2,true,origin,origin+Vector3.right*.1f);
            Packet(1,false,origin,origin,true);
            executor.AdvanceTrajectory(.025f,100025);
            Require(Vector3.Distance(executor.transform.position,origin+Vector3.right*.05f)<.0001f,
                "Approved movement is interpolated; a delayed duplicate stop cannot cancel the new chain.");
            Packet(3,false,origin+Vector3.right*.1f,origin+new Vector3(.1f,0,.1f));
            executor.AdvanceTrajectory(.05f,100075);
            Require(Vector3.Distance(executor.transform.position,origin+new Vector3(.1f,0,.05f))<.0001f,
                "Queued movement follows both legs of a corner without a shortcut.");
            Vector3 expiredPosition=executor.transform.position;
            executor.AdvanceTrajectory(.1f,100200);
            Require(executor.transform.position==expiredPosition &&
                    executor.TrajectoryStatusJson(100200).Contains("\"stopped\":true"),
                "Expiry cancels movement and reports a stationary recovery anchor.");
            Packet(4,true,expiredPosition+Vector3.right*.01f,expiredPosition+Vector3.right*.1f);
            executor.AdvanceTrajectory(.05f,100050);
            Require(executor.transform.position==expiredPosition,
                "A restart from delayed feedback cannot create an unapproved connector.");
            Packet(5,false,expiredPosition,expiredPosition,true);
            Packet(6,true,expiredPosition,expiredPosition+Vector3.right*.1f,epochOffset:1);
            executor.AdvanceTrajectory(.05f,100050);
            Require(executor.transform.position==expiredPosition,"Wrong-reset packets never move the drone.");
            Packet(6,true,expiredPosition,expiredPosition+Vector3.right*.1f);
            Packet(8,false,expiredPosition+Vector3.right*.1f,expiredPosition+Vector3.right*.2f);
            executor.AdvanceTrajectory(.05f,100050);
            Require(executor.transform.position==expiredPosition,"Sequence gaps cancel the pending chain.");
            Packet(9,false,expiredPosition,expiredPosition,true);
            Packet(10,true,expiredPosition,expiredPosition+Vector3.right*.2f);
            Packet(11,false,expiredPosition+Vector3.right*.2f,expiredPosition+Vector3.right*.3f);
            executor.AdvanceTrajectory(.05f,100050);
            Require(executor.transform.position==expiredPosition,"Outstanding approved distance stays bounded.");
            Packet(12,false,expiredPosition,expiredPosition,true);
            Packet(13,true,expiredPosition,expiredPosition+Vector3.right*.1f);
            executor.ResetForTrial();
            executor.AdvanceTrajectory(.05f,100050);
            Require(executor.transform.position==expiredPosition,"Physical reset clears pending interpolation.");
            Debug.Log("NAVIGATION_SMOOTHNESS_CHECKS_PASS");
        }
        finally
        {
            executor.ResetForTrial();
            executor.transform.position=saved;
            Physics.SyncTransforms();
        }
    }

    [MenuItem("Drone Delivery/Verify Hardcoded A to B Flight")]
    public static void BeginFlight()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play first.");
        if (Application.isBatchMode) UnityEditor.SceneManagement.EditorSceneManager.OpenScene(TwoWarehouseCourseBuilder.ScenePath);
        TwoWarehouseCourseBuilder.Verify();
        SessionState.SetBool("TwoWarehouse.FlightVerification",true);
        SessionState.SetBool(Running,true);
        EditorApplication.isPlaying=true;
    }

    static IEnumerator CheckFlight()
    {
        Time.timeScale=4;
        double deadline=EditorApplication.timeSinceStartup+90;
        var flight=UnityEngine.Object.FindAnyObjectByType<HardcodedWarehouseFlight>();
        Require(flight != null && flight.enabled,"Explicit flight fixture enables the local demonstration.");
        var course=flight.GetComponent<WarehouseCourse>();
        bool carried=false;
        yield return null;
        while(flight.IsFlying && EditorApplication.timeSinceStartup<deadline)
        {
            carried |= course.drone.HeldPackage==course.package;
            RequireNoContact(course);
            yield return null;
        }
        Require(!flight.IsFlying && flight.Status=="Delivered in Warehouse B","Automatic flight reaches its completed delivery state. Status="+flight.Status);
        Require(carried,"Automatic flight picked up and carried the package.");
        Require(course.Delivered && course.ContainsPackage() && !course.drone.HasPackage,"Automatic flight releases and settles the package inside Warehouse B.");
        Require(!course.CollisionObserved,"Smooth flight completes the full route without obstacle contacts.");
        Vector3 finished=flight.transform.position;
        flight.GetComponent<DroneCommandSubscriber>().SendMessage("ReceivePosition",new PointMsg(0,0,0));
        Require(flight.transform.position==finished,"ROS position callbacks cannot override local flight control.");
        flight.StartFlight();
        deadline=EditorApplication.timeSinceStartup+10;
        while(!course.drone.HasPackage && flight.IsFlying && EditorApplication.timeSinceStartup<deadline) yield return null;
        Require(course.drone.HasPackage,"The completed flight can restart and pick up again.");
        course.ResetTrial();
        Vector3 reset=flight.transform.position;
        double wait=EditorApplication.timeSinceStartup+.5;
        while(EditorApplication.timeSinceStartup<wait) yield return null;
        Require(!flight.IsFlying && flight.transform.position==reset && !course.drone.HasPackage,"Reset cancels local movement and releases its package.");
    }
    static void RequireNoContact(WarehouseCourse course)
    {
        if(course.CollisionObserved) throw new InvalidOperationException("Automatic flight contact: "+course.ContactName);
    }
    static void Tick()
    {
        if(!SessionState.GetBool(Running,false) || !EditorApplication.isPlaying || EditorApplication.isPaused) return;
        try
        {
            if(run==null) { checks.Clear(); failed=false; run=SessionState.GetBool("TwoWarehouse.FlightVerification",false) ? CheckFlight() : Check(); }
            if(run.MoveNext()) return;
        }
        catch(Exception e) { failed=true; checks.Add("FAIL: "+e); Debug.LogException(e); }
        File.WriteAllLines(SessionState.GetBool("TwoWarehouse.FlightVerification",false) ? "Library/hardcoded-flight-checks.txt" : "Library/course-play-checks.txt",checks);
        Debug.Log(failed ? "TWO_WAREHOUSE_PLAY_CHECKS_FAILED" : "TWO_WAREHOUSE_PLAY_CHECKS_PASS");
        SessionState.SetBool(Running,false); run=null;
        Time.timeScale=1;
        EditorApplication.isPlaying=false;
        if (Application.isBatchMode) EditorApplication.Exit(failed ? 1 : 0);
    }
    static void Require(bool condition,string message)
    {
        if(!condition) throw new InvalidOperationException(message);
        checks.Add("PASS: "+message);
    }
    static IEnumerator Check()
    {
        double wait=EditorApplication.timeSinceStartup+1;
        while(EditorApplication.timeSinceStartup<wait) yield return null;
        var course=UnityEngine.Object.FindAnyObjectByType<WarehouseCourse>();
        var drone=course.drone; var package=course.package;
        var command=drone.GetComponent<DroneCommandSubscriber>();
        var bridge=drone.GetComponent<PackageActionBridge>();
        Vector3 initialDrone=drone.transform.position, initialPackage=GameObject.Find("PickupPoint").transform.position;
        var localFlight=drone.GetComponent<HardcodedWarehouseFlight>();
        Require(localFlight != null && !localFlight.enabled && !localFlight.IsFlying,
            "Saved course defaults to ROS control with local flight disabled.");
        Require(command.enabled && bridge.enabled && drone.transform.position==GameObject.Find("DroneStart").transform.position,
            "Without ROS commands the drone waits at its start with both handlers enabled.");
        Require(!course.CollisionObserved,"Initial drone and package have no contacts.");
        Send(command,new Vector3(float.NaN,1,1));
        Require(drone.transform.position==initialDrone,"Non-finite ROS position is rejected.");

        var common=new GameObject("Sensor common root fixture");
        Transform oldParent=drone.transform.parent;
        drone.transform.SetParent(common.transform,true);
        drone.transform.position=new Vector3(-200,4,-200);
        var obstacle=GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name="Thin wall fixture"; obstacle.transform.SetParent(common.transform,true);
        obstacle.transform.position=drone.transform.position+Vector3.right*3;
        obstacle.transform.localScale=new Vector3(.1f,5,5);
        Physics.SyncTransforms();
        var sensor=drone.GetComponent<DroneObstacleSensorPublisher>();
        float range=(float)typeof(DroneObstacleSensorPublisher).GetMethod("MeasureDistance",BindingFlags.NonPublic|BindingFlags.Instance)
            .Invoke(sensor,new object[]{drone.transform.position,Vector3.right});
        Require(range>2 && range<3,"Distance sensor sees a sibling obstacle under a shared root.");
        Send(command,drone.transform.position+Vector3.right*6);
        Require(course.CollisionObserved,"Swept collision reports crossing a thin wall between ROS commands.");
        drone.transform.SetParent(oldParent,true);
        UnityEngine.Object.DestroyImmediate(common);
        course.ResetTrial();
        Require(!course.CollisionObserved && drone.transform.position==initialDrone && package.transform.position==initialPackage,
            "Reset restores initial transforms and clears observations.");
        Send(command,package.transform.position-(drone.packageMount.position-drone.transform.position));
        bridge.SendMessage("OnPackageAction",new StringMsg("PICKUP"));
        wait=EditorApplication.timeSinceStartup+4;
        while(drone.IsBusy && EditorApplication.timeSinceStartup<wait) yield return null;
        Require(!Physics.ComputePenetration(drone.GetComponent<BoxCollider>(),drone.transform.position,drone.transform.rotation,
            package.GetComponent<BoxCollider>(),package.transform.position,package.transform.rotation,out _,out _),
            "Drone body collision leaves the package mount unobstructed.");
        Require(drone.HeldPackage==package,"Real gripper closes and attaches the package through the ROS action handler.");
        Vector3[] route={new Vector3(34,3,55),new Vector3(34,3,71),new Vector3(25,3,71),new Vector3(25,3,87),new Vector3(43,3,87),new Vector3(43,3,103),new Vector3(34,3,103),new Vector3(34,3,119.5f)};
        foreach(Vector3 point in route)
        {
            Send(command,point); wait=EditorApplication.timeSinceStartup+.15;
            while(EditorApplication.timeSinceStartup<wait) yield return null;
            Require(!course.CollisionObserved,"Carried fixture segment is clear at "+point);
        }
        Require(drone.HeldPackage==package,"Package stays attached through both doorways and the yard bypass.");
        Send(command,new Vector3(34, .65f-(drone.packageMount.position.y-drone.transform.position.y),119.5f));
        bridge.SendMessage("OnPackageAction",new StringMsg("DROP"));
        Require(package.Owner==null && !drone.HasPackage,"ROS drop releases the physical package.");
        Require(!course.Delivered,"Release alone does not confirm delivery.");
        wait=EditorApplication.timeSinceStartup+6;
        while(!course.Delivered && EditorApplication.timeSinceStartup<wait) yield return null;
        Require(course.Delivered && course.ContainsPackage(),"Whole package settles inside Warehouse B before delivery confirmation. Position="+package.transform.position+" velocity="+package.GetComponent<Rigidbody>().linearVelocity);
        Require(!course.CollisionObserved,"Physical carry and release finish without obstacle contacts.");
        package.transform.position=new Vector3(course.deliveryVolume.bounds.max.x-.05f,package.transform.position.y,119.5f);
        Physics.SyncTransforms();
        Require(!course.ContainsPackage(),"A package straddling the delivery boundary is rejected.");
        course.ResetTrial();
        Send(command,package.transform.position-(drone.packageMount.position-drone.transform.position));
        bridge.SendMessage("OnPackageAction",new StringMsg("PICKUP"));
        Require(drone.IsBusy,"Reset permits a fresh PICKUP action.");
        course.ResetTrial();
        wait=EditorApplication.timeSinceStartup+1;
        while(EditorApplication.timeSinceStartup<wait) yield return null;
        Require(!drone.HasPackage && !drone.IsBusy && package.Owner==null,"Reset during gripper closure cancels the pending attachment.");

        using(var capture=new RgbDepthCapture(DroneRgbCamera.EnsureOnDrone(drone.gameObject)))
        {
            var frame=capture.Capture();
            Require(frame.Rgb.Length==frame.Width*frame.Height*3 && frame.DepthMetres.Length==frame.Width*frame.Height,
                "HDRP RGB and metric depth capture return matching image dimensions.");
            Require(frame.Rgb.Max()>frame.Rgb.Min() && frame.DepthMetres.Any(d=>float.IsFinite(d) && d>0),
                "RGB contains scene detail and metric depth contains finite surface measurements.");
        }
        var views=Camera.main.GetComponent<WarehouseCourseViews>();
        for(int view=0;view<3;view++)
        {
            views.SelectView(view);
            wait=EditorApplication.timeSinceStartup+.5;
            while(EditorApplication.timeSinceStartup<wait) yield return null;
            Capture(Camera.main,"Library/course-view-"+view+".png");
        }
        CheckSensorViews(views,drone);
        Require(course.ResetSequence==3,"Repeatable resets expose a monotonically increasing trial sequence.");
    }

    static void CheckSensorViews(WarehouseCourseViews views,PackageHandling drone)
    {
        var sensor=DroneRgbCamera.EnsureOnDrone(drone.gameObject);
        Vector3 originalPosition=sensor.transform.localPosition;
        Quaternion originalRotation=sensor.transform.localRotation;
        int roofLayer=LayerMask.NameToLayer(WarehouseCourseViews.RoofLayerName);
        var probe=GameObject.CreatePrimitive(PrimitiveType.Cube);
        probe.name="Sensor roof visibility fixture";
        probe.layer=roofLayer;
        probe.GetComponent<Renderer>().sharedMaterial=AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/CourseWall.mat");
        // Isolate the roof probe from the drone body, which occludes an upward camera.
        sensor.transform.position=new Vector3(-200,10,-200);
        sensor.transform.rotation=Quaternion.LookRotation(Vector3.up,Vector3.forward);
        probe.transform.position=sensor.transform.position+Vector3.up*3;
        probe.transform.localScale=new Vector3(4,.2f,4);
        try
        {
            using(var capture=new RgbDepthCapture(sensor))
            {
                var baseline=capture.Capture();
                int centre=(baseline.Height/2)*baseline.Width+baseline.Width/2;
                Require(Mathf.Abs(baseline.DepthMetres[centre]-2.9f)<.05f,
                    "Sensor detects roof-layer geometry at the expected metric depth: "+baseline.DepthMetres[centre]);
                for(int view=0;view<3;view++)
                {
                    views.SelectView(view);
                    var frame=capture.Capture();
                    Require((sensor.SensorCamera.cullingMask & (1<<roofLayer))!=0 &&
                        views.roofs.All(r=>r==null || r.enabled) &&
                        Mathf.Abs(frame.DepthMetres[centre]-baseline.DepthMetres[centre])<.001f,
                        "Spectator view "+view+" preserves roof visibility and sensor depth.");
                    Require(Enumerable.Range(0,3).All(channel=>
                        Mathf.Abs(frame.Rgb[centre*3+channel]-baseline.Rgb[centre*3+channel])<=3),
                        "Spectator view "+view+" preserves the stationary RGB roof measurement.");
                    bool spectatorSeesRoof=(Camera.main.cullingMask & (1<<roofLayer))!=0;
                    Require(spectatorSeesRoof==(view==1),"Only spectator cutaway views exclude the roof layer.");
                }
            }
        }
        finally
        {
            sensor.transform.localPosition=originalPosition;
            sensor.transform.localRotation=originalRotation;
            UnityEngine.Object.DestroyImmediate(probe);
        }
    }
    static void Send(DroneCommandSubscriber command,Vector3 p)
    { command.SendMessage("ReceivePosition",new PointMsg(p.x,p.z,p.y)); }
    static void Capture(Camera camera,string path)
    {
        var target=new RenderTexture(1280,720,24);
        var image=new Texture2D(1280,720,TextureFormat.RGB24,false);
        var previous=RenderTexture.active;
        try
        {
            target.Create();
            RenderPipeline.SubmitRenderRequest(camera,new RenderPipeline.StandardRequest{destination=target});
            RenderTexture.active=target; image.ReadPixels(new Rect(0,0,1280,720),0,0); image.Apply();
            File.WriteAllBytes(path,image.EncodeToPNG());
        }
        finally { RenderTexture.active=previous; target.Release(); UnityEngine.Object.DestroyImmediate(image); UnityEngine.Object.DestroyImmediate(target); }
    }
}
