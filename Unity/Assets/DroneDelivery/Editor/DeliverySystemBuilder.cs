using System.IO;
using DroneDelivery;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace DroneDeliveryEditor {
public static class DeliverySystemBuilder {
 const string Folder = "Assets/DroneDelivery";
 static Material shell, carbon, metal, rubber, orange, cyan, red, cardboard, tape, white, floor;
 [MenuItem("Drone Delivery/Build Test Scene and Prefabs")]
 public static void Build() {
  if (EditorSceneManager.GetActiveScene().isDirty) throw new System.InvalidOperationException("Save current scene first.");
  Directory.CreateDirectory(Folder + "/Materials"); Directory.CreateDirectory("Assets/Prefabs"); Directory.CreateDirectory("Assets/Scenes");
  shell = Mat("Ceramic body", new Color(.73f,.76f,.77f), .45f, .55f);
  carbon = Mat("Graphite structure", new Color(.055f,.065f,.075f), .65f,.38f);
  metal = Mat("Machined aluminum", new Color(.42f,.46f,.5f), .85f,.7f);
  rubber = Mat("Rubber", new Color(.022f,.025f,.03f), 0,.2f);
  orange = Mat("Safety orange", new Color(.95f,.24f,.045f), .2f,.4f);
  cyan = Mat("Navigation cyan", new Color(.02f,.7f,.85f), .25f,.65f);
  red = Mat("Navigation red", new Color(.8f,.035f,.025f), .2f,.5f);
  cardboard = Mat("Kraft cardboard", new Color(.53f,.34f,.16f),0,.12f);
  tape = Mat("Packing tape", new Color(.72f,.53f,.28f),0,.38f);
  white = Mat("Shipping label",new Color(.87f,.86f,.8f),0,.15f);
  floor = Mat("Concrete",new Color(.16f,.19f,.22f),.1f,.22f);
  var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
  var package = Package();
  PrefabUtility.SaveAsPrefabAsset(package.gameObject,"Assets/Prefabs/Package.prefab");
  Object.DestroyImmediate(package.gameObject);
  var drone = Drone();
  PrefabUtility.SaveAsPrefabAsset(drone.gameObject,"Assets/Prefabs/Drone.prefab");
  Object.DestroyImmediate(drone.gameObject);
  drone = ((GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Drone.prefab"))).GetComponent<PackageHandling>();
  drone.transform.position = new Vector3(-3,3.05f,-2);
  package = ((GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Package.prefab"))).GetComponent<DeliveryPackage>();
  package.transform.position = new Vector3(-3,.3f,0);
  var ground = Part("Test floor",null,new Vector3(0,-.1f,0),new Vector3(18,.2f,12),floor); ground.AddComponent<BoxCollider>();
  for(int i=-8;i<=8;i+=2) Part("Floor joint",null,new Vector3(i,.002f,0),new Vector3(.012f,.004f,12),carbon);
  for(int i=-6;i<=6;i+=2) Part("Floor joint",null,new Vector3(0,.002f,i),new Vector3(18,.004f,.012f),carbon);
  Pad(-3,"PICKUP",orange); Pad(3,"DELIVERY",cyan);
  var light = new GameObject("Key light").AddComponent<Light>(); light.type=LightType.Directional; light.intensity=1.6f; light.shadows=LightShadows.Soft; light.transform.rotation=Quaternion.Euler(48,-30,0);
  RenderSettings.ambientMode=UnityEngine.Rendering.AmbientMode.Trilight; RenderSettings.ambientSkyColor=new Color(.48f,.55f,.65f); RenderSettings.ambientEquatorColor=new Color(.24f,.28f,.34f); RenderSettings.ambientGroundColor=new Color(.1f,.12f,.15f);
  var cam = new GameObject("Main Camera",typeof(Camera),typeof(AudioListener)); cam.tag="MainCamera"; cam.transform.position=new Vector3(8,6,-11); cam.transform.LookAt(new Vector3(0,1,0));
  cam.GetComponent<Camera>().fieldOfView=48; cam.GetComponent<Camera>().backgroundColor=new Color(.07f,.1f,.15f); cam.GetComponent<Camera>().clearFlags=CameraClearFlags.SolidColor;
  var demo = new GameObject("Delivery demonstration").AddComponent<DeliveryDemo>(); demo.drone=drone; demo.package=package;
  EditorSceneManager.SaveScene(scene,"Assets/Scenes/DroneGripperTest.unity"); AssetDatabase.SaveAssets();
  Debug.Log("DRONE_DELIVERY_BUILD_COMPLETE");
 }
 static Material Mat(string name,Color color,float metallic,float smooth) {
  string path=Folder+"/Materials/"+name+".mat";
  var m=AssetDatabase.LoadAssetAtPath<Material>(path);
  if(!m) { m=new Material(Shader.Find("Standard")); AssetDatabase.CreateAsset(m,path); }
  m.color=color; m.SetFloat("_Metallic",metallic); m.SetFloat("_Glossiness",smooth); EditorUtility.SetDirty(m); return m;
 }
 static Transform Empty(string name,Transform parent,Vector3 pos) { var t=new GameObject(name).transform; t.SetParent(parent,false); t.localPosition=pos; return t; }
 static GameObject Part(string name,Transform parent,Vector3 pos,Vector3 scale,Material mat,PrimitiveType type=PrimitiveType.Cube) {
  var o=GameObject.CreatePrimitive(type); o.name=name; o.transform.SetParent(parent,false); o.transform.localPosition=pos; o.transform.localScale=scale; o.GetComponent<Renderer>().sharedMaterial=mat; Object.DestroyImmediate(o.GetComponent<Collider>()); return o;
 }
 static void Rod(string name,Transform parent,Vector3 a,Vector3 b,float radius,Material mat) {
  var o=Part(name,parent,(a+b)*.5f,new Vector3(radius*2,(b-a).magnitude*.5f,radius*2),mat,PrimitiveType.Cylinder); o.transform.localRotation=Quaternion.FromToRotation(Vector3.up,b-a);
 }
 static void Text(string content,Transform parent,Vector3 p,Quaternion r,float size,Color c) {
  var t=Empty("Label "+content,parent,p); t.localRotation=r; var m=t.gameObject.AddComponent<TextMesh>(); m.text=content; m.fontSize=80; m.characterSize=size*.22f; m.anchor=TextAnchor.MiddleCenter; m.color=c;
 }
 static DeliveryPackage Package() {
  var root=Empty("Package",null,Vector3.zero);
  var box=Part("Corrugated cardboard box",root,Vector3.zero,new Vector3(.6f,.6f,.6f),cardboard);
  var col=root.gameObject.AddComponent<BoxCollider>(); col.size=Vector3.one*.6f;
  var rb=root.gameObject.AddComponent<Rigidbody>(); rb.mass=1; rb.interpolation=RigidbodyInterpolation.Interpolate; rb.collisionDetectionMode=CollisionDetectionMode.ContinuousDynamic;
  Part("Top tape",root,new Vector3(0,.301f,0),new Vector3(.13f,.004f,.603f),tape);
  Part("Front tape",root,new Vector3(0,0,-.301f),new Vector3(.13f,.6f,.003f),tape);
  Part("Back tape",root,new Vector3(0,0,.301f),new Vector3(.13f,.6f,.003f),tape);
  Part("Top seam",root,new Vector3(0,.304f,0),new Vector3(.004f,.002f,.6f),cardboard);
  Part("Shipping label",root,new Vector3(.16f,.307f,.055f),new Vector3(.16f,.002f,.24f),white);
  for(int i=0;i<17;i++) Part("Barcode",root,new Vector3(.10f+i*.007f,.309f,.09f),new Vector3(i%3==0?.004f:.002f,.002f,.09f),carbon);
  Text("CARGO / 01",root,new Vector3(0,.09f,-.304f),Quaternion.identity,.035f,new Color(.12f,.1f,.06f));
  Text("THIS SIDE UP",root,new Vector3(0,-.12f,-.304f),Quaternion.identity,.022f,new Color(.12f,.1f,.06f));
  return root.gameObject.AddComponent<DeliveryPackage>();
 }
 static PackageHandling Drone() {
  var root=Empty("DroneRoot",null,Vector3.zero); var model=Empty("DroneModel",root,Vector3.zero);
  Part("Lower chassis",model,Vector3.zero,new Vector3(.78f,.19f,1.04f),carbon);
  Part("Rounded upper shell",model,new Vector3(0,.12f,0),new Vector3(.81f,.30f,1.09f),shell,PrimitiveType.Sphere);
  Part("Battery enclosure",model,new Vector3(0,.24f,-.07f),new Vector3(.43f,.13f,.63f),carbon);
  Part("Battery latch",model,new Vector3(0,.315f,-.12f),new Vector3(.16f,.025f,.13f),orange);
  for(int s=-1;s<=1;s+=2) {
   Part("Orange identification stripe",model,new Vector3(s*.30f,.17f,0),new Vector3(.075f,.035f,.58f),orange);
   for(int i=0;i<7;i++) Part("Cooling vent",model,new Vector3(s*.395f,.03f,-.27f+i*.065f),new Vector3(.012f,.07f,.025f),rubber);
   Rod("Landing rail",model,new Vector3(s*.66f,-.70f,-.61f),new Vector3(s*.66f,-.70f,.61f),.032f,carbon);
   for(int z=-1;z<=1;z+=2) {
    Rod("Landing leg",model,new Vector3(s*.29f,-.07f,z*.33f),new Vector3(s*.66f,-.68f,z*.43f),.025f,metal);
    Part("Skid rubber foot",model,new Vector3(s*.66f,-.72f,z*.46f),new Vector3(.10f,.065f,.20f),rubber);
    Vector3 hub=new Vector3(s*.91f,.02f,z*.91f);
    Rod("Carbon arm",model,new Vector3(s*.25f,0,z*.27f),hub,.055f,carbon);
    Rod("Arm reinforcement",model,new Vector3(s*.24f,-.07f,z*.20f),hub+Vector3.down*.06f,.025f,metal);
    Part("Motor mount",model,hub,new Vector3(.24f,.06f,.24f),metal);
    Part("Brushless motor",model,hub+Vector3.up*.10f,new Vector3(.18f,.07f,.18f),carbon,PrimitiveType.Cylinder);
    Part("Motor crown",model,hub+Vector3.up*.175f,new Vector3(.17f,.015f,.17f),metal,PrimitiveType.Cylinder);
    for(int k=0;k<8;k++) { float a=k*Mathf.PI/4; Part("Motor cooling slot",model,hub+new Vector3(Mathf.Cos(a)*.091f,.1f,Mathf.Sin(a)*.091f),new Vector3(.018f,.07f,.018f),rubber); }
    var rotor=Empty("Propeller",model,hub+Vector3.up*.21f); rotor.localRotation=Quaternion.Euler(0,s*z*28,0); rotor.gameObject.AddComponent<DeliveryRotor>().speed=s*z*1500;
    var blade=Part("Composite propeller",rotor,Vector3.zero,new Vector3(.78f,.016f,.07f),carbon); blade.transform.localRotation=Quaternion.Euler(7,0,0);
    Part("Orange blade tip",rotor,new Vector3(.355f,0,0),new Vector3(.07f,.018f,.071f),orange);
    Part("Orange blade tip",rotor,new Vector3(-.355f,0,0),new Vector3(.07f,.018f,.071f),orange);
    Part("Hub nut",rotor,Vector3.up*.025f,new Vector3(.065f,.03f,.065f),metal,PrimitiveType.Cylinder);
    Part("Navigation lamp",model,hub+new Vector3(s*.12f,-.02f,0),new Vector3(.045f,.045f,.07f),z>0?cyan:red,PrimitiveType.Sphere);
   }
  }
  Rod("GPS mast",model,new Vector3(0,.26f,.31f),new Vector3(0,.44f,.31f),.014f,metal);
  Part("GPS puck",model,new Vector3(0,.46f,.31f),new Vector3(.18f,.026f,.18f),shell,PrimitiveType.Cylinder);
  Rod("Radio antenna",model,new Vector3(.22f,.17f,-.36f),new Vector3(.25f,.52f,-.40f),.009f,rubber);
  Part("Forward sensor",model,new Vector3(0,-.03f,-.55f),new Vector3(.25f,.13f,.12f),carbon);
  for(int s=-1;s<=1;s+=2) Part("Optical lens",model,new Vector3(s*.067f,-.03f,-.618f),new Vector3(.067f,.067f,.012f),cyan,PrimitiveType.Sphere);
  Text("ATLAS  /  01",model,new Vector3(0,.18f,-.41f),Quaternion.Euler(55,0,0),.033f,Color.black);
  for(int s=-1;s<=1;s+=2) for(int z=-1;z<=1;z+=2) Part("Chassis fastener",model,new Vector3(s*.33f,.155f,z*.36f),new Vector3(.025f,.013f,.025f),metal,PrimitiveType.Cylinder);
  var grip=Empty("Gripper",root,new Vector3(0,-.38f,0));
  Rod("Suspension shaft",grip,new Vector3(0,.29f,0),new Vector3(0,0,0),.075f,metal);
  Part("Actuator housing",grip,Vector3.zero,new Vector3(.63f,.16f,.23f),carbon);
  Part("Actuator cover",grip,new Vector3(0,-.005f,-.124f),new Vector3(.38f,.10f,.018f),orange);
  Rod("Linear slide front",grip,new Vector3(-.50f,-.12f,-.08f),new Vector3(.50f,-.12f,-.08f),.021f,metal);
  Rod("Linear slide rear",grip,new Vector3(-.50f,-.12f,.08f),new Vector3(.50f,-.12f,.08f),.021f,metal);
  var gc=grip.gameObject.AddComponent<GripperController>();
  for(int s=-1;s<=1;s+=2) {
   var jaw=Empty(s<0?"LeftJaw":"RightJaw",grip,new Vector3(s*.46f,-.12f,0));
   Part("Slide carriage",jaw,Vector3.zero,new Vector3(.12f,.12f,.27f),metal);
   Part("Claw finger",jaw,new Vector3(0,-.27f,0),new Vector3(.045f,.48f,.12f),metal);
   Part("Rubber contact pad",jaw,new Vector3(-s*.012f,-.40f,0),new Vector3(.024f,.20f,.16f),rubber);
   for(int i=0;i<5;i++) Part("Grip rib",jaw,new Vector3(-s*.028f,-.32f-i*.036f,0),new Vector3(.008f,.012f,.16f),carbon);
   if(s<0) gc.leftJaw=jaw; else gc.rightJaw=jaw;
  }
  var mount=Empty("PackageMount",grip,new Vector3(0,-.57f,0));
  var detector=Empty("PickupDetector",root,new Vector3(0,-.95f,0));
  var handling=root.gameObject.AddComponent<PackageHandling>(); handling.packageMount=mount; handling.pickupDetector=detector; handling.gripper=gc;
  var body=root.gameObject.AddComponent<Rigidbody>(); body.isKinematic=true; body.useGravity=false;
  var collider=root.gameObject.AddComponent<BoxCollider>(); collider.size=new Vector3(.8f,.3f,1.1f); collider.center=new Vector3(0,.07f,0);
  return handling;
 }
 static void Pad(float x,string label,Material mat) {
  for(int s=-1;s<=1;s+=2) {
   Part("Pad marking",null,new Vector3(x+s*.9f,.008f,0),new Vector3(.065f,.009f,1.8f),mat);
   Part("Pad marking",null,new Vector3(x,.008f,s*.9f),new Vector3(1.8f,.009f,.065f),mat);
  }
  Text(label,null,new Vector3(x,.015f,-1.25f),Quaternion.Euler(90,0,0),.12f,Color.white);
 }
}
}
