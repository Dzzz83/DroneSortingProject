using System.Collections;
using UnityEngine;
namespace DroneDelivery {
public class DeliveryDemo : MonoBehaviour {
 const string WarehouseDroneName = "DeliveryDrone_ATLAS01";
 const string WarehousePackageName = "DeliveryPackage_Pickup";
 const string WarehouseRootName = "Member2_DroneSystem";
 const string WarehouseDeliveryTargetName = "SortingZoneA";
 public PackageHandling drone;
 public DeliveryPackage package;
 public Vector3 delivery = new Vector3(3, .3f, 0);
 public float speed = 1.6f;
 public float rotationSpeed = 4f;
 public string Phase { get; private set; } = "Ready";
 public bool Completed { get; private set; }
 public bool Passed { get; private set; }

 [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
 static void InstallWarehouseMission() {
  GameObject droneObject = GameObject.Find(WarehouseDroneName);
  GameObject packageObject = GameObject.Find(WarehousePackageName);
  GameObject deliveryTarget = GameObject.Find(WarehouseDeliveryTargetName);
  if (!droneObject || !packageObject || !deliveryTarget) return;

  PackageHandling handling = droneObject.GetComponent<PackageHandling>();
  DeliveryPackage deliveryPackage = packageObject.GetComponent<DeliveryPackage>();
  if (!handling || !deliveryPackage) {
   Debug.LogError("DELIVERY_DEMO: Warehouse drone or package component is missing");
   return;
  }

  GameObject root = GameObject.Find(WarehouseRootName);
  if (!root) root = droneObject;
  DeliveryDemo demo = root.GetComponent<DeliveryDemo>();
  if (!demo) demo = root.AddComponent<DeliveryDemo>();
  demo.drone = handling;
  demo.package = deliveryPackage;
  demo.delivery = new Vector3(deliveryTarget.transform.position.x, packageObject.transform.position.y, deliveryTarget.transform.position.z);
  demo.speed = 4f;
 }

 IEnumerator Start() {
  if (!drone || !package || !drone.packageMount) { Fail("Mission references are missing"); yield break; }
  yield return new WaitForSeconds(1);
  Vector3 offset = drone.transform.position - drone.packageMount.position;
  Phase = "01  APPROACH"; yield return Move(package.transform.position + offset + Vector3.up * 1.8f);
  Phase = "02  DESCEND"; yield return Move(package.transform.position + offset);
  Phase = "03  GRIP";
  if (!drone.PickUpPackage()) { Fail("Package not detected"); yield break; }
  yield return new WaitUntil(() => !drone.IsBusy);
  if (!drone.HasPackage) { Fail("Attachment failed"); yield break; }
  Phase = "04  LIFT"; yield return Move(drone.transform.position + Vector3.up * 1.8f);
  Phase = "05  CARRY"; yield return Move(delivery + offset + Vector3.up * 1.8f);
  if (Vector3.Distance(package.transform.position, drone.packageMount.position) > .01f) { Fail("Carry drift"); yield break; }
  Phase = "06  LOWER"; yield return Move(delivery + offset);
  Phase = "07  RELEASE";
  if (!drone.DropPackage()) { Fail("Release failed"); yield break; }
  yield return new WaitForSeconds(.7f);
  yield return Move(drone.transform.position + Vector3.up * 1.8f);
  yield return new WaitForSeconds(1);
  Passed = !drone.HasPackage && package.Owner == null && Vector3.Distance(package.transform.position, delivery) < .08f;
  Completed = true; Phase = Passed ? "DELIVERY COMPLETE" : "DELIVERY CHECK FAILED";
  Debug.Log("DELIVERY_DEMO: " + Phase + " | package=" + package.transform.position);
 }
 IEnumerator Move(Vector3 destination) {
  while (Vector3.Distance(drone.transform.position, destination) > .005f) {
   Vector3 direction = destination - drone.transform.position;
   direction.y = 0f;
   if (direction.sqrMagnitude > .001f) {
    Quaternion targetRotation = Quaternion.LookRotation(-direction.normalized, Vector3.up);
    drone.transform.rotation = Quaternion.Slerp(drone.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
   }
   drone.transform.position = Vector3.MoveTowards(drone.transform.position, destination, speed * Time.deltaTime);
   yield return null;
  }
  drone.transform.position = destination;
 }
 void Fail(string reason) { Phase = reason; Completed = true; Passed = false; Debug.LogError("DELIVERY_DEMO: " + reason); }
 void OnGUI() {
  GUI.Box(new Rect(20,20,340,115), "");
  GUI.Label(new Rect(36,30,310,25), "D R O N E   /   C A R G O   S Y S T E M");
  GUI.Label(new Rect(36,60,310,25), Phase);
  GUI.Label(new Rect(36,90,310,25), "MILESTONE 01    |    " + drone.Status);
 }
}
}
