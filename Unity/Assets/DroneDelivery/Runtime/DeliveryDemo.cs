using System.Collections;
using UnityEngine;
namespace DroneDelivery {
public class DeliveryDemo : MonoBehaviour {
 public PackageHandling drone;
 public DeliveryPackage package;
 public Vector3 delivery = new Vector3(3, .3f, 0);
 public float speed = 1.6f;
 public string Phase { get; private set; } = "Ready";
 public bool Completed { get; private set; }
 public bool Passed { get; private set; }
 IEnumerator Start() {
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
