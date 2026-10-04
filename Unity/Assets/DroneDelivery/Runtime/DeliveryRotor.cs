using UnityEngine;
namespace DroneDelivery {
public class DeliveryRotor : MonoBehaviour {
 public float speed = 1500;
 void Update() { transform.Rotate(0, speed * Time.deltaTime, 0, Space.Self); }
}
}
