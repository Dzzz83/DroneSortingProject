using UnityEngine;

namespace DroneSim
{
    public sealed class DroneCameraFollow : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private Vector3 closeOffset = new Vector3(0f, 3f, -8f);
        [SerializeField] private Vector3 farOffset = new Vector3(0f, 8f, -15f);
        [SerializeField] private float positionSmooth = 7f;
        [SerializeField] private float rotationSmooth = 9f;

        private bool farMode;

        public void SetTarget(Transform newTarget)
        {
            target = newTarget;
        }

        private void LateUpdate()
        {
            if (!target) return;

            if (Input.GetKeyDown(KeyCode.C))
            {
                farMode = !farMode;
            }

            Vector3 offset = farMode ? farOffset : closeOffset;
            Vector3 desiredPosition = target.TransformPoint(offset);
            transform.position = Vector3.Lerp(transform.position, desiredPosition, 1f - Mathf.Exp(-positionSmooth * Time.deltaTime));

            Quaternion desiredRotation = Quaternion.LookRotation(target.position + Vector3.up * 1.2f - transform.position, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, 1f - Mathf.Exp(-rotationSmooth * Time.deltaTime));
        }
    }
}
