using UnityEngine;

namespace DroneSim
{
    [RequireComponent(typeof(Rigidbody))]
    public sealed class DroneController : MonoBehaviour
    {
        [Header("Lift")]
        [SerializeField] private float hoverThrottle = 0.58f;
        [SerializeField] private float liftPower = 24f;
        [SerializeField] private float verticalDamping = 2.8f;

        [Header("Attitude")]
        [SerializeField] private float pitchTorque = 7f;
        [SerializeField] private float rollTorque = 7f;
        [SerializeField] private float yawTorque = 4f;
        [SerializeField] private float angularDamping = 3.5f;
        [SerializeField] private float selfLevelStrength = 3.2f;

        [Header("Input")]
        [SerializeField] private float throttleResponse = 0.7f;
        [SerializeField] private KeyCode resetKey = KeyCode.R;

        private Rigidbody body;
        private Vector3 spawnPosition;
        private Quaternion spawnRotation;
        private float throttle;

        public float Throttle => throttle;
        public float Altitude => transform.position.y;
        public float Speed => body ? body.linearVelocity.magnitude : 0f;

        private void Awake()
        {
            body = GetComponent<Rigidbody>();
            body.mass = 1.2f;
            body.linearDamping = 0.08f;
            body.angularDamping = 0.25f;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            spawnPosition = transform.position;
            spawnRotation = transform.rotation;
            throttle = hoverThrottle;
        }

        private void Update()
        {
            float throttleInput = ReadThrottleInput();
            throttle = Mathf.Clamp01(throttle + throttleInput * throttleResponse * Time.deltaTime);

            if (Input.GetKeyDown(resetKey))
            {
                ResetDrone();
            }
        }

        private void FixedUpdate()
        {
            float pitch = Input.GetAxisRaw("Vertical");
            float roll = -Input.GetAxisRaw("Horizontal");
            float yaw = ReadYawInput();

            Vector3 lift = transform.up * (throttle * liftPower);
            Vector3 damping = -Vector3.Project(body.linearVelocity, transform.up) * verticalDamping;
            body.AddForce(lift + damping, ForceMode.Force);

            body.AddRelativeTorque(new Vector3(pitch * pitchTorque, yaw * yawTorque, roll * rollTorque), ForceMode.Force);
            body.AddTorque(-body.angularVelocity * angularDamping, ForceMode.Force);
            ApplySelfLeveling();
        }

        public void ResetDrone()
        {
            transform.SetPositionAndRotation(spawnPosition, spawnRotation);
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            throttle = hoverThrottle;
        }

        private float ReadThrottleInput()
        {
            float input = 0f;
            if (Input.GetKey(KeyCode.Space)) input += 1f;
            if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)) input -= 1f;
            return input;
        }

        private float ReadYawInput()
        {
            float input = 0f;
            if (Input.GetKey(KeyCode.E)) input += 1f;
            if (Input.GetKey(KeyCode.Q)) input -= 1f;
            return input;
        }

        private void ApplySelfLeveling()
        {
            Vector3 correction = Vector3.Cross(transform.up, Vector3.up);
            body.AddTorque(correction * selfLevelStrength, ForceMode.Force);
        }
    }
}
