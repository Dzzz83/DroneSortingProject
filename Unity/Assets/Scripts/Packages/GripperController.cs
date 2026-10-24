using UnityEngine;

namespace DroneDelivery
{
    public class GripperController : MonoBehaviour
    {
        public Transform leftJaw;
        public Transform rightJaw;
        public float openWidth = 0.46f;
        public float closedWidth = 0.325f;
        public float travelSpeed = 0.4f;

        public bool IsClosed { get; private set; }
        public bool AtTarget => leftJaw &&
            Mathf.Abs(Mathf.Abs(leftJaw.localPosition.x) - TargetWidth) < 0.002f;

        private float TargetWidth => IsClosed ? closedWidth : openWidth;

        public void Open() => IsClosed = false;
        public void Close() => IsClosed = true;

        private void Update()
        {
            MoveJaw(leftJaw, -TargetWidth);
            MoveJaw(rightJaw, TargetWidth);
        }

        private void MoveJaw(Transform jaw, float targetX)
        {
            if (!jaw)
                return;

            Vector3 position = jaw.localPosition;
            position.x = Mathf.MoveTowards(position.x, targetX, travelSpeed * Time.deltaTime);
            jaw.localPosition = position;
        }
    }
}
