using System.Collections;
using UnityEngine;

namespace DroneDelivery
{
    public class PackageHandling : MonoBehaviour
    {
        public Transform packageMount;
        public Transform pickupDetector;
        public GripperController gripper;
        public float pickupRadius = 0.22f;
        public LayerMask packageLayers = ~0;

        public DeliveryPackage HeldPackage { get; private set; }
        public bool HasPackage => HeldPackage != null;
        public bool IsBusy { get; private set; }
        public string Status { get; private set; } = "Ready";

        public DeliveryPackage DetectPackage()
        {
            if (!pickupDetector)
                return null;

            DeliveryPackage closestPackage = null;
            float closestDistance = pickupRadius;
            Collider[] colliders = Physics.OverlapSphere(pickupDetector.position,
                pickupRadius, packageLayers, QueryTriggerInteraction.Ignore);
            foreach (Collider collider in colliders)
            {
                DeliveryPackage package = collider.GetComponentInParent<DeliveryPackage>();
                if (!package || package.Owner != null)
                    continue;

                float distance = Vector3.Distance(packageMount.position, package.transform.position);
                if (distance <= closestDistance)
                {
                    closestDistance = distance;
                    closestPackage = package;
                }
            }
            return closestPackage;
        }

        public bool PickUpPackage()
        {
            if (!isActiveAndEnabled || IsBusy || HasPackage ||
                !packageMount || !pickupDetector || !gripper)
                return false;

            DeliveryPackage package = DetectPackage();
            if (!package)
            {
                Status = "No package in range";
                return false;
            }
            IsBusy = true;
            StartCoroutine(Pickup(package));
            return true;
        }

        private IEnumerator Pickup(DeliveryPackage package)
        {
            Status = "Closing gripper";
            gripper.Close();
            yield return new WaitUntil(() => gripper.AtTarget);

            if (package && Vector3.Distance(packageMount.position, package.transform.position) <= pickupRadius &&
                package.Attach(this, packageMount))
            {
                HeldPackage = package;
                Status = "Carrying";
            }
            else
            {
                gripper.Open();
                Status = "Pickup cancelled";
            }
            IsBusy = false;
        }

        public bool DropPackage()
        {
            if (!isActiveAndEnabled || IsBusy || !HasPackage)
                return false;

            HeldPackage.Release();
            HeldPackage = null;
            gripper.Open();
            Status = "Released";
            return true;
        }

        private void OnDisable()
        {
            StopAllCoroutines();
            IsBusy = false;
            if (HeldPackage)
            {
                HeldPackage.Release();
                HeldPackage = null;
            }
            if (gripper)
                gripper.Open();
        }

        private void OnDrawGizmosSelected()
        {
            if (!pickupDetector)
                return;
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(pickupDetector.position, pickupRadius);
        }
    }
}
