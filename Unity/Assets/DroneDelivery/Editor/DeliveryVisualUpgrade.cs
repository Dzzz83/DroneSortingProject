using System;
using System.IO;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

namespace DroneDeliveryEditor {
public static class DeliveryVisualUpgrade {
 const string Root="Assets/DroneDelivery/HD";
 static Dictionary<string,Material> materials=new Dictionary<string,Material>();
 static int meshIndex;
 [MenuItem("Drone Delivery/Upgrade Visuals to HD")]
 public static void Upgrade() {
  if(EditorApplication.isPlaying) throw new InvalidOperationException("Exit Play mode first.");
  if(EditorSceneManager.GetActiveScene().isDirty) throw new InvalidOperationException("Save the current scene before upgrading.");
  Directory.CreateDirectory(Root+"/Meshes"); Directory.CreateDirectory(Root+"/Materials"); Directory.CreateDirectory(Root+"/Textures");
  materials.Clear(); meshIndex=0;
  TextureSet("Carbon",0); TextureSet("Cardboard",1); TextureSet("Metal",2);
  UpgradePrefab("Assets/Prefabs/Drone.prefab",true);
  UpgradePrefab("Assets/Prefabs/Package.prefab",false);
  Lighting();
  EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
  EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene()); AssetDatabase.SaveAssets();
  Debug.Log("ATLAS_HD_UPGRADE_COMPLETE");
 }
 static void UpgradePrefab(string path,bool drone) {
  var root=PrefabUtility.LoadPrefabContents(path);
  try {
   if(root.transform.Find("HD visual revision")) return;
   foreach(var filter in root.GetComponentsInChildren<MeshFilter>(true)) {
    var mesh=filter.sharedMesh; if(!mesh) continue;
    var t=filter.transform; string name=t.name;
    if(name=="Composite propeller") {
     filter.sharedMesh=SaveMesh(Propeller(),"Swept airfoil"); t.localScale=Vector3.one; t.localRotation=Quaternion.identity;
    } else if(name=="Orange blade tip") { t.gameObject.SetActive(false); }
    else if(mesh.name=="Cube" && t.localScale.x>.022f && t.localScale.y>.022f && t.localScale.z>.022f) {
     Vector3 size=t.localScale;
     float radius=Mathf.Min(size.x,Mathf.Min(size.y,size.z))*.17f;
     if(name=="Corrugated cardboard box") radius=.006f;
     filter.sharedMesh=SaveMesh(RoundedBox(size,radius),name); t.localScale=Vector3.one;
    } else if(mesh.name=="Sphere" && name=="Rounded upper shell") {
     filter.sharedMesh=SaveMesh(RoundedBox(new Vector3(.82f,.245f,1.09f),.115f),"Sculpted shell"); t.localScale=Vector3.one;
    } else if(mesh.name=="Cylinder") filter.sharedMesh=GetCylinder();
    var renderer=filter.GetComponent<Renderer>();
    if(renderer && renderer.sharedMaterial) renderer.sharedMaterial=MaterialFor(renderer.sharedMaterial);
   }
   if(drone) AddDetails(root.transform);
   new GameObject("HD visual revision").transform.SetParent(root.transform,false);
   PrefabUtility.SaveAsPrefabAsset(root,path);
  } finally { PrefabUtility.UnloadPrefabContents(root); }
 }
 static Material MaterialFor(Material source) {
  string name=source.name; Material m;
  if(materials.TryGetValue(name,out m)) return m;
  string path=Root+"/Materials/"+name+" HD.mat";
  m=AssetDatabase.LoadAssetAtPath<Material>(path);
  if(!m) { m=new Material(source); AssetDatabase.CreateAsset(m,path); }
  string texture=null; float bump=.15f;
  if(name.Contains("Graphite")) { texture="Carbon"; m.color=new Color(.24f,.27f,.30f); m.SetFloat("_Metallic",.25f); m.SetFloat("_Glossiness",.43f); m.mainTextureScale=new Vector2(3,3); m.SetTextureScale("_BumpMap",new Vector2(3,3)); bump=.18f; }
  if(name.Contains("cardboard")) { texture="Cardboard"; m.color=Color.white; m.SetFloat("_Glossiness",.15f); bump=.3f; }
  if(name.Contains("aluminum")) { texture="Metal"; m.color=new Color(.65f,.69f,.74f); m.SetFloat("_Metallic",.85f); m.SetFloat("_Glossiness",.62f); bump=.12f; }
  if(name.Contains("Ceramic")) { m.color=new Color(.67f,.72f,.75f); m.SetFloat("_Metallic",.3f); m.SetFloat("_Glossiness",.68f); }
  if(name.Contains("Rubber")) { m.color=new Color(.035f,.04f,.045f); m.SetFloat("_Glossiness",.24f); }
  if(name.Contains("Navigation")) { m.EnableKeyword("_EMISSION"); m.SetColor("_EmissionColor",m.color*1.4f); }
  if(texture!=null) {
   m.mainTexture=AssetDatabase.LoadAssetAtPath<Texture2D>(Root+"/Textures/"+texture+"_Albedo.png");
   m.SetTexture("_BumpMap",AssetDatabase.LoadAssetAtPath<Texture2D>(Root+"/Textures/"+texture+"_Normal.png")); m.SetFloat("_BumpScale",bump); m.EnableKeyword("_NORMALMAP");
  }
  materials[name]=m; EditorUtility.SetDirty(m); return m;
 }
 static void TextureSet(string name,int kind) {
  const int n=2048; var color=new Color32[n*n]; var normal=new Color32[n*n];
  var rng=new System.Random(731+kind);
  for(int y=0;y<n;y++) for(int x=0;x<n;x++) {
   float noise=(float)rng.NextDouble(); float v;
   if(kind==0) {
    int cx=x/64,cy=y/64; bool horizontal=((cx+cy)%4)<2;
    float u=(horizontal?y:x)%64/64f; float edge=Mathf.Pow(Mathf.Sin(u*Mathf.PI),.4f);
    float fiber=.5f+.5f*Mathf.Sin((horizontal?y:x)*Mathf.PI*.5f);
    v=.28f+.30f*edge+.09f*fiber+.04f*noise;
    color[y*n+x]=new Color(v,v*1.025f,v*1.06f,1);
    float slope=Mathf.Cos(u*Mathf.PI)*.13f;
    normal[y*n+x]=new Color(.5f+(horizontal?0:slope),.5f+(horizontal?slope:0),1,1);
   } else if(kind==1) {
    v=(noise-.5f)*.085f+.014f*Mathf.Sin(y*.7f)+.012f*Mathf.Sin(x*.037f);
    color[y*n+x]=new Color(.57f+v,.39f+v,.215f+v,1);
    normal[y*n+x]=new Color(.5f+(noise-.5f)*.16f,.5f+(float)(rng.NextDouble()-.5)*.16f,1,1);
   } else {
    v=.62f+.08f*Mathf.Sin(y*2.13f)+.045f*noise;
    color[y*n+x]=new Color(v,v,v,1);
    normal[y*n+x]=new Color(.5f,.5f+.08f*Mathf.Cos(y*2.13f),1,1);
   }
  }
  WriteTexture(name+"_Albedo",color,n,false); WriteTexture(name+"_Normal",normal,n,true);
 }
 static void WriteTexture(string name,Color32[] pixels,int n,bool normal) {
  string path=Root+"/Textures/"+name+".png";
  var t=new Texture2D(n,n,TextureFormat.RGBA32,false,normal); t.SetPixels32(pixels); t.Apply(); File.WriteAllBytes(path,t.EncodeToPNG()); Object.DestroyImmediate(t);
  AssetDatabase.ImportAsset(path);
  var importer=(TextureImporter)AssetImporter.GetAtPath(path); importer.textureType=normal?TextureImporterType.NormalMap:TextureImporterType.Default; importer.maxTextureSize=2048; importer.anisoLevel=8; importer.mipmapEnabled=true; importer.textureCompression=TextureImporterCompression.CompressedHQ; importer.SaveAndReimport();
 }
 static Mesh SaveMesh(Mesh mesh,string label) {
  mesh.name=label+" HD"; string path=Root+"/Meshes/"+(meshIndex++).ToString("D3")+".asset";
  var old=AssetDatabase.LoadAssetAtPath<Mesh>(path);
  if(old) { EditorUtility.CopySerialized(mesh,old); Object.DestroyImmediate(mesh); return old; }
  AssetDatabase.CreateAsset(mesh,path); return mesh;
 }
 static float[] Coordinates(float half,float r) {
  return new[]{-half,-half+r*.07612f,-half+r*.29289f,-half+r*.61732f,-half+r,half-r,half-r*.61732f,half-r*.29289f,half-r*.07612f,half};
 }
 static Mesh RoundedBox(Vector3 size,float r) {
  var vertices=new List<Vector3>(); var normals=new List<Vector3>(); var uvs=new List<Vector2>(); var tris=new List<int>();
  Vector3 h=size*.5f,inner=h-Vector3.one*r;
  Vector3[] axes={Vector3.right,Vector3.up,Vector3.forward};
  for(int axis=0;axis<3;axis++) for(int sign=-1;sign<=1;sign+=2) {
   int a=(axis+1)%3,b=(axis+2)%3; float[] us=Coordinates(h[a],r),vs=Coordinates(h[b],r); int start=vertices.Count;
   for(int j=0;j<vs.Length;j++) for(int i=0;i<us.Length;i++) {
    Vector3 p=axes[axis]*h[axis]*sign+axes[a]*us[i]+axes[b]*vs[j];
    Vector3 clamped=new Vector3(Mathf.Clamp(p.x,-inner.x,inner.x),Mathf.Clamp(p.y,-inner.y,inner.y),Mathf.Clamp(p.z,-inner.z,inner.z));
    Vector3 normal=(p-clamped).normalized; vertices.Add(clamped+normal*r); normals.Add(normal); uvs.Add(new Vector2(us[i]/size[a]+.5f,vs[j]/size[b]+.5f));
   }
   for(int j=0;j<vs.Length-1;j++) for(int i=0;i<us.Length-1;i++) {
    int p=start+j*us.Length+i,q=p+1,s=p+us.Length,t=s+1;
    if(sign>0) tris.AddRange(new[]{p,q,t,p,t,s}); else tris.AddRange(new[]{p,t,q,p,s,t});
   }
  }
  var mesh=new Mesh(); mesh.SetVertices(vertices); mesh.SetNormals(normals); mesh.SetUVs(0,uvs); mesh.SetTriangles(tris,0); mesh.RecalculateTangents(); mesh.RecalculateBounds(); return mesh;
 }
 static Mesh GetCylinder() {
  string path=Root+"/Meshes/PrecisionCylinder.asset"; var mesh=AssetDatabase.LoadAssetAtPath<Mesh>(path); if(mesh) return mesh;
  var v=new List<Vector3>();var uv=new List<Vector2>();var tr=new List<int>();
  float[] ys={-1,-1,-.98f,-.94f,-.90f,.90f,.94f,.98f,1,1};
  float[] rs={0,.46f,.48f,.495f,.5f,.5f,.495f,.48f,.46f,0}; int seg=64;
  for(int j=0;j<ys.Length;j++) for(int i=0;i<=seg;i++) {float a=i*Mathf.PI*2/seg;v.Add(new Vector3(Mathf.Cos(a)*rs[j],ys[j],Mathf.Sin(a)*rs[j])); uv.Add(new Vector2((float)i/seg,(ys[j]+1)*.5f));}
  for(int j=0;j<ys.Length-1;j++) for(int i=0;i<seg;i++) {int a=j*(seg+1)+i,b=a+seg+1;tr.AddRange(new[]{a,b,a+1,a+1,b,b+1});}
  mesh=new Mesh();mesh.name="64 segment machined cylinder";mesh.SetVertices(v);mesh.SetUVs(0,uv);mesh.SetTriangles(tr,0);mesh.RecalculateNormals();mesh.RecalculateTangents();mesh.RecalculateBounds();AssetDatabase.CreateAsset(mesh,path);return mesh;
 }
 static Mesh Propeller() {
  var v=new List<Vector3>();var uv=new List<Vector2>();var tr=new List<int>(); int span=32,section=16;
  for(int side=0;side<2;side++) {
   int offset=v.Count;float s=side==0?1:-1;
   for(int i=0;i<=span;i++) {
    float t=(float)i/span;float x=.035f+.38f*t;float chord=.045f+.045f*Mathf.Sin(Mathf.PI*t);chord*=1-.72f*Mathf.Pow(t,6);float sweep=.036f*t*t;float twist=Mathf.Lerp(22,7,t)*Mathf.Deg2Rad;
    for(int j=0;j<=section;j++) {float a=j*Mathf.PI*2/section;float z=Mathf.Cos(a)*chord*.5f;float y=Mathf.Sin(a)*chord*.065f;v.Add(new Vector3(s*x,y*Mathf.Cos(twist)-z*Mathf.Sin(twist),s*(z*Mathf.Cos(twist)+y*Mathf.Sin(twist)+sweep)));uv.Add(new Vector2(t,(float)j/section));}
   }
   for(int i=0;i<span;i++) for(int j=0;j<section;j++) {int a=offset+i*(section+1)+j,b=a+section+1;tr.AddRange(new[]{a,b,a+1,a+1,b,b+1});}
  }
  var mesh=new Mesh();mesh.SetVertices(v);mesh.SetUVs(0,uv);mesh.SetTriangles(tr,0);mesh.RecalculateNormals();mesh.RecalculateTangents();mesh.RecalculateBounds();return mesh;
 }
 static GameObject Detail(string name,Transform parent,Vector3 p,Vector3 size,Material mat) {
  var o=new GameObject(name,typeof(MeshFilter),typeof(MeshRenderer));o.transform.SetParent(parent,false);o.transform.localPosition=p;o.GetComponent<MeshFilter>().sharedMesh=SaveMesh(RoundedBox(size,Mathf.Min(size.x,Mathf.Min(size.y,size.z))*.18f),name);o.GetComponent<MeshRenderer>().sharedMaterial=mat;return o;
 }
 static Material Find(string name) {return materials[name];}
 static void AddDetails(Transform root) {
  var model=root.Find("DroneModel");
  Material dark=Find("Graphite structure"),metal=Find("Machined aluminum"),orange=Find("Safety orange"),rubber=Find("Rubber");
  for(int s=-1;s<=1;s+=2) {
   Detail("Shell seam",model,new Vector3(s*.383f,.105f,0),new Vector3(.007f,.012f,.79f),rubber);
   Detail("Battery retaining strap",model,new Vector3(s*.135f,.312f,-.07f),new Vector3(.028f,.016f,.58f),rubber);
   Detail("Arm root flange",model,new Vector3(s*.39f,-.015f,0),new Vector3(.07f,.23f,.43f),metal);
   for(int z=-1;z<=1;z+=2) {
    Vector3 hub=new Vector3(s*.91f,.02f,z*.91f);
    for(int k=0;k<12;k++) {
     float a=k*Mathf.PI/6; var fin=Detail("Motor heat sink fin",model,hub+new Vector3(Mathf.Cos(a)*.087f,.10f,Mathf.Sin(a)*.087f),new Vector3(.009f,.102f,.026f),metal);fin.transform.localRotation=Quaternion.Euler(0,-a*Mathf.Rad2Deg,0);
    }
    for(int a=-1;a<=1;a+=2) for(int b=-1;b<=1;b+=2) {
     var bolt=new GameObject("Socket head fastener",typeof(MeshFilter),typeof(MeshRenderer));bolt.transform.SetParent(model,false);bolt.transform.localPosition=hub+new Vector3(a*.084f,.038f,b*.084f);bolt.transform.localScale=new Vector3(.019f,.008f,.019f);bolt.GetComponent<MeshFilter>().sharedMesh=GetCylinder();bolt.GetComponent<Renderer>().sharedMaterial=metal;
     Detail("Fastener recess",model,hub+new Vector3(a*.084f,.047f,b*.084f),new Vector3(.007f,.001f,.007f),rubber);
    }
   }
  }
  var grip=root.Find("Gripper");
  for(int s=-1;s<=1;s+=2) {
   var jaw=grip.Find(s<0?"LeftJaw":"RightJaw");
   Detail("Finger reinforcing spine",jaw,new Vector3(s*.027f,-.20f,0),new Vector3(.018f,.31f,.065f),dark);
   Detail("Jaw warning band",jaw,new Vector3(0,-.23f,-.063f),new Vector3(.047f,.041f,.006f),orange);
  }
  // Mesh cable loops remain visual-only and do not affect grasping.
  for(int s=-1;s<=1;s+=2) for(int i=0;i<14;i++) {
   float a=i*Mathf.PI/13; Vector3 p=new Vector3(s*(.15f+.04f*Mathf.Sin(a)),-.23f+.10f*Mathf.Cos(a),.08f);
   var o=GameObject.CreatePrimitive(PrimitiveType.Sphere);o.name="Actuator cable";o.transform.SetParent(grip,false);o.transform.localPosition=p;o.transform.localScale=Vector3.one*.018f;o.GetComponent<Renderer>().sharedMaterial=rubber;Object.DestroyImmediate(o.GetComponent<Collider>());
  }
 }
 public static void Lighting() {
  var key=GameObject.Find("Key light");if(key) {var l=key.GetComponent<Light>();l.intensity=1.1f;l.shadowStrength=.7f;l.shadowBias=.03f;l.shadowNormalBias=.15f;}
  AddLight("HD fill",new Vector3(-3,5,-4),new Color(.65f,.8f,1),2.3f);
  AddLight("HD rim",new Vector3(2,5,4),new Color(1,.87f,.69f),3.2f);
  RenderSettings.ambientSkyColor=new Color(.40f,.46f,.56f);RenderSettings.ambientEquatorColor=new Color(.23f,.26f,.30f);RenderSettings.ambientGroundColor=new Color(.10f,.12f,.15f);
  var camera=Camera.main;if(camera) {camera.allowHDR=true;camera.allowMSAA=true;}
  var existing=GameObject.Find("HD reflection probe");if(!existing) {
   var probe=new GameObject("HD reflection probe").AddComponent<ReflectionProbe>();probe.transform.position=new Vector3(0,2,0);probe.size=new Vector3(18,10,12);probe.resolution=256;probe.mode=UnityEngine.Rendering.ReflectionProbeMode.Realtime;probe.refreshMode=UnityEngine.Rendering.ReflectionProbeRefreshMode.OnAwake;probe.clearFlags=UnityEngine.Rendering.ReflectionProbeClearFlags.Skybox;probe.intensity=.75f;
  }
 }
 static void AddLight(string name,Vector3 pos,Color color,float power) {
  var o=GameObject.Find(name);if(!o)o=new GameObject(name);o.transform.position=pos;o.transform.LookAt(new Vector3(0,1,0));var l=o.GetComponent<Light>();if(!l)l=o.AddComponent<Light>();l.type=LightType.Point;l.range=15;l.intensity=power;l.color=color;l.shadows=LightShadows.None;
 }
 public static void RenderHero(string outputPath) {
  var drone=Object.FindFirstObjectByType<DroneDelivery.PackageHandling>();var camera=Camera.main;
  Vector3 focus=drone.transform.position+Vector3.down*.25f;
  var previousPosition=camera.transform.position;var previousRotation=camera.transform.rotation;float previousFov=camera.fieldOfView;
  camera.transform.position=focus+new Vector3(2.2f,1.4f,-3.2f);camera.transform.LookAt(focus);camera.fieldOfView=42;
  var rt=new RenderTexture(2560,1600,24,RenderTextureFormat.ARGB32);rt.antiAliasing=8;var old=RenderTexture.active;
  var image=new Texture2D(2560,1600,TextureFormat.RGB24,false);
  try {camera.targetTexture=rt;camera.Render();RenderTexture.active=rt;image.ReadPixels(new Rect(0,0,2560,1600),0,0);image.Apply();File.WriteAllBytes(outputPath,image.EncodeToPNG());}
  finally {camera.targetTexture=null;RenderTexture.active=old;rt.Release();Object.DestroyImmediate(rt);Object.DestroyImmediate(image);camera.transform.SetPositionAndRotation(previousPosition,previousRotation);camera.fieldOfView=previousFov;}
 }
}
}


