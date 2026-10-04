using UnityEngine;

namespace DroneSim
{
    public sealed class SpinRotors : MonoBehaviour
    {
        [SerializeField] private float degreesPerSecond = 1500f;

        private void Update()
        {
            transform.Rotate(Vector3.up, degreesPerSecond * Time.deltaTime, Space.Self);
        }
    }
}
