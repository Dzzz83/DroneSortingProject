using UnityEngine;
namespace DroneDelivery {
[RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
public class DeliveryPackage : MonoBehaviour {
 public PackageHandling Owner { get; private set; }
 Rigidbody body; Transform oldParent; bool oldGravity, oldKinematic; RigidbodyInterpolation oldInterpolation;
 void Awake() { body = GetComponent<Rigidbody>(); }
 public bool Attach(PackageHandling owner, Transform mount) {
  if (Owner != null || owner == null || mount == null) return false;
  if (!body) body = GetComponent<Rigidbody>();
  oldInterpolation = body.interpolation; body.interpolation = RigidbodyInterpolation.None; oldParent = transform.parent; oldGravity = body.useGravity; oldKinematic = body.isKinematic;
  if (!body.isKinematic) { body.linearVelocity = Vector3.zero; body.angularVelocity = Vector3.zero; }
  body.isKinematic = true; body.useGravity = false; Owner = owner;
  transform.SetParent(mount, false); transform.localPosition = Vector3.zero; transform.localRotation = Quaternion.identity;
  return true;
 }
 public void Release() {
  if (Owner == null) return;
  transform.SetParent(oldParent, true); Owner = null;
  body.isKinematic = oldKinematic; body.useGravity = oldGravity; body.interpolation = oldInterpolation;
  if (!body.isKinematic) { body.linearVelocity = Vector3.zero; body.angularVelocity = Vector3.zero; }
 }
}
}
