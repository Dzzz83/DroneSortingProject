using UnityEngine;

namespace DroneDelivery
{
    [RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
    public class DeliveryPackage : MonoBehaviour
    {
        public PackageHandling Owner { get; private set; }

        private Rigidbody body;
        private Transform originalParent;
        private bool originalGravity;
        private bool originalKinematic;
        private RigidbodyInterpolation originalInterpolation;

        private void Awake()
        {
            body = GetComponent<Rigidbody>();
        }

        public bool Attach(PackageHandling owner, Transform mount)
        {
            if (Owner != null || owner == null || mount == null)
                return false;
            if (!body)
                body = GetComponent<Rigidbody>();

            // Restore the package's original physics settings when it is released.
            originalParent = transform.parent;
            originalGravity = body.useGravity;
            originalKinematic = body.isKinematic;
            originalInterpolation = body.interpolation;
            body.interpolation = RigidbodyInterpolation.None;
            if (!body.isKinematic)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
            body.isKinematic = true;
            body.useGravity = false;
            Owner = owner;
            transform.SetParent(mount, false);
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
            return true;
        }

        public void Release()
        {
            if (Owner == null)
                return;

            transform.SetParent(originalParent, true);
            // A ROS move and release can arrive before the next physics step.
            Physics.SyncTransforms();
            Owner = null;
            body.isKinematic = originalKinematic;
            body.useGravity = originalGravity;
            body.interpolation = originalInterpolation;
            if (!body.isKinematic)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
    }
}
