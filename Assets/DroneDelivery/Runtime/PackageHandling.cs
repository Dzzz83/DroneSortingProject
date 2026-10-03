using System.Collections;
using UnityEngine;
namespace DroneDelivery {
public class PackageHandling : MonoBehaviour {
 public Transform packageMount, pickupDetector;
 public GripperController gripper;
 public float pickupRadius = .22f;
 public LayerMask packageLayers = ~0;
 public DeliveryPackage HeldPackage { get; private set; }
 public bool HasPackage { get { return HeldPackage != null; } }
 public bool IsBusy { get; private set; }
 public string Status { get; private set; } = "Ready";
 public DeliveryPackage DetectPackage() {
  if (!pickupDetector) return null;
  DeliveryPackage best = null; float distance = pickupRadius;
  foreach (Collider c in Physics.OverlapSphere(pickupDetector.position, pickupRadius, packageLayers, QueryTriggerInteraction.Ignore)) {
   var p = c.GetComponentInParent<DeliveryPackage>();
   if (!p || p.Owner != null) continue;
   float d = Vector3.Distance(packageMount.position, p.transform.position);
   if (d <= distance) { distance = d; best = p; }
  }
  return best;
 }
 public bool PickUpPackage() {
  if (!isActiveAndEnabled || IsBusy || HasPackage || !packageMount || !pickupDetector || !gripper) return false;
  var package = DetectPackage();
  if (!package) { Status = "No package in range"; return false; }
  IsBusy = true; StartCoroutine(Pickup(package)); return true;
 }
 IEnumerator Pickup(DeliveryPackage package) {
  Status = "Closing gripper"; gripper.Close();
  yield return new WaitUntil(() => gripper.AtTarget);
  if (package && Vector3.Distance(packageMount.position, package.transform.position) <= pickupRadius && package.Attach(this, packageMount)) {
   HeldPackage = package; Status = "Carrying";
  } else { gripper.Open(); Status = "Pickup cancelled"; }
  IsBusy = false;
 }
 public bool DropPackage() {
  if (!isActiveAndEnabled || IsBusy || !HasPackage) return false;
  HeldPackage.Release(); HeldPackage = null; gripper.Open(); Status = "Released"; return true;
 }
 void OnDisable() {
  StopAllCoroutines(); IsBusy = false;
  if (HeldPackage) { HeldPackage.Release(); HeldPackage = null; }
  if (gripper) gripper.Open();
 }
 void OnDrawGizmosSelected() {
  if (!pickupDetector) return;
  Gizmos.color = Color.cyan; Gizmos.DrawWireSphere(pickupDetector.position, pickupRadius);
 }
}
}