using UnityEngine;

namespace DroneSim
{
    public sealed class DroneGate : MonoBehaviour
    {
        [SerializeField] private DroneHud hud;
        [SerializeField] private Color activeColor = new Color(0f, 0.9f, 1f, 0.45f);
        [SerializeField] private Color clearedColor = new Color(0.1f, 1f, 0.25f, 0.25f);

        private Renderer[] gateRenderers;
        private bool cleared;

        public void SetHud(DroneHud newHud)
        {
            hud = newHud;
        }

        private void Awake()
        {
            gateRenderers = GetComponentsInChildren<Renderer>();
            ApplyColor(activeColor);
        }

        private void OnTriggerEnter(Collider other)
        {
            if (cleared || !other.GetComponentInParent<DroneController>()) return;

            cleared = true;
            hud?.AddGate();
            ApplyColor(clearedColor);
        }

        private void ApplyColor(Color color)
        {
            if (gateRenderers == null) return;

            foreach (Renderer gateRenderer in gateRenderers)
            {
                gateRenderer.material.color = color;
            }
        }
    }
}
