using UnityEngine;
namespace DroneDelivery {
public class GripperController : MonoBehaviour {
 public Transform leftJaw, rightJaw;
 public float openWidth = .46f, closedWidth = .325f, travelSpeed = .4f;
 public bool IsClosed { get; private set; }
 public bool AtTarget { get { return leftJaw && Mathf.Abs(Mathf.Abs(leftJaw.localPosition.x) - (IsClosed ? closedWidth : openWidth)) < .002f; } }
 public void Open() { IsClosed = false; }
 public void Close() { IsClosed = true; }
 void Update() {
  float width = IsClosed ? closedWidth : openWidth;
  Move(leftJaw, -width); Move(rightJaw, width);
 }
 void Move(Transform jaw, float x) {
  if (!jaw) return;
  Vector3 p = jaw.localPosition; p.x = Mathf.MoveTowards(p.x, x, travelSpeed * Time.deltaTime); jaw.localPosition = p;
 }
}
}