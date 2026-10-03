using UnityEngine;
using UnityEngine.UI;

namespace DroneSim
{
    public sealed class DroneHud : MonoBehaviour
    {
        [SerializeField] private DroneController drone;
        [SerializeField] private Text readout;

        public int GatesCleared { get; private set; }

        public void SetDrone(DroneController newDrone)
        {
            drone = newDrone;
        }

        public void SetReadout(Text newReadout)
        {
            readout = newReadout;
        }

        public void AddGate()
        {
            GatesCleared += 1;
        }

        private void Update()
        {
            if (!drone || !readout) return;

            readout.text =
                $"Throttle: {drone.Throttle:0.00}\n" +
                $"Altitude: {drone.Altitude:0.0} m\n" +
                $"Speed: {drone.Speed:0.0} m/s\n" +
                $"Gates: {GatesCleared}";
        }
    }
}
