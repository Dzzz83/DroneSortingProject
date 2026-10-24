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
        if (SessionState.GetBool(Running,false) && !SessionState.GetBool("TwoWarehouse.FlightVerification",false))
        {
            var flight=UnityEngine.Object.FindAnyObjectByType<HardcodedWarehouseFlight>();
            if(flight != null) flight.enabled=false;
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
        Require(flight != null && flight.enabled,"Saved scene enables the automatic hardcoded flight.");
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
            typeof(WarehouseCourseViews).GetField("view",BindingFlags.NonPublic|BindingFlags.Instance).SetValue(views,view);
            wait=EditorApplication.timeSinceStartup+.5;
            while(EditorApplication.timeSinceStartup<wait) yield return null;
            Capture(Camera.main,"Library/course-view-"+view+".png");
        }
        Require(course.ResetSequence==3,"Repeatable resets expose a monotonically increasing trial sequence.");
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
