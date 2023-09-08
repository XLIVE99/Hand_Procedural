using UnityEngine;
using Extension;

namespace BIK
{
    //Simple third person camera controller
    [RequireComponent(typeof(Camera))]
    public class CameraController : MonoBehaviour
    {
        //Offsets
        [SerializeField] private TrackInfo positionTrack;
        [SerializeField] private TrackInfo lookTrack;

        //Zoom parameters
        [SerializeField, Space(5)] private float distance = 8f;
        [SerializeField] private Vector2 distanceClamp;
        [SerializeField] private float zoomDistance = 4f;

        //Vertical rotation limit
        [SerializeField, Space(5)] private Vector2 verticalLimit; //Local positionTrack target limit

        //Other parameters
        [SerializeField, Space(5)] private float sensitivity = 3f;
        [SerializeField] private float speed = 0.3f;
        [SerializeField] private LayerMask collisionLayers;

        [System.Serializable]
        public struct TrackInfo
        {
            public Transform target;
            public Vector3 globalOffset;
            public Vector3 localOffset;
        }

        private Vector3 direction;

        private Camera cam;
        private float raySphereRadius;
        private bool isZoom = false;
        private bool isLocked = true;

        #region SINGLETON
        public static CameraController instance;
        private void Awake()
        {
            if (instance == null)
            {
                instance = this;
            }
            else
                Destroy(gameObject);

            cam = GetComponent<Camera>();
        }
        #endregion

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            if (positionTrack.target == null || lookTrack.target == null)
                return;

            float radius = 0f;
            float offsetY = 0f;
            //Draw top point of camera range
            radius = Mathf.Cos(Mathf.Min(verticalLimit.y, 90f) * Mathf.Deg2Rad) * distance;
            offsetY = Mathf.Sin(Mathf.Min(verticalLimit.y, 90f) * Mathf.Deg2Rad) * distance;
            GizmosDrawCircle(CalculateTrackPoint(positionTrack) + positionTrack.target.up * offsetY, radius, positionTrack.target.up, Color.cyan);

            //Middle point of camera range
            GizmosDrawCircle(CalculateTrackPoint(positionTrack), distance, positionTrack.target.up, Color.black);

            //Bottom point of camera range
            radius = Mathf.Cos(Mathf.Min(-verticalLimit.x, 90f) * Mathf.Deg2Rad) * distance;
            offsetY = Mathf.Sin(Mathf.Min(-verticalLimit.x, 90f) * Mathf.Deg2Rad) * distance;
            GizmosDrawCircle(CalculateTrackPoint(positionTrack) - positionTrack.target.up * offsetY, radius, positionTrack.target.up, Color.blue);
        }

        private void GizmosDrawCircle(Vector3 center, float radius, Vector3 normal, Color c)
        {
            Gizmos.color = c;
            Vector3 tangent = Vector3.Cross(normal, Vector3.up);
            if (Vector3.Cross(normal, Vector3.forward).magnitude > tangent.magnitude)
                tangent = Vector3.Cross(normal, Vector3.forward);
            tangent.Normalize();

            for(int i = 0; i < 360; i++)
            {
                Gizmos.DrawLine(center + Quaternion.AngleAxis(i, normal) * tangent * radius,
                    center + Quaternion.AngleAxis(i + 1, normal) * tangent * radius);
            }
        }

        [ContextMenu("Update current position")]
        private void UpdatePosition()
        {
            direction = (transform.position - CalculateTrackPoint(positionTrack)).normalized;

            //Clamp direction depend on Target (Method commented below)
            direction = VectorExtensions.ClampAngleAxis(direction, positionTrack.target.up, verticalLimit.x, verticalLimit.y);

            //Camera zoom in and out
            float currentDistance = 0f;
            if (isZoom)
                currentDistance = zoomDistance;
            else
            {
                distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel") * 5f, distanceClamp.x, distanceClamp.y);
                currentDistance = distance;
            }

            //Set camera position
            Vector3 localDirection = transform.position - CalculateTrackPoint(positionTrack);

            //Collision detection
            if (Physics.SphereCast(CalculateTrackPoint(positionTrack), raySphereRadius, direction, out RaycastHit hitInfo, distance, collisionLayers))
                currentDistance = hitInfo.distance;

            Vector3 slerpDirection = Vector3.Slerp(localDirection, direction * currentDistance, Time.deltaTime * speed);

            transform.position = CalculateTrackPoint(positionTrack) + slerpDirection;

            //Camera rotation
            transform.LookAt(CalculateTrackPoint(lookTrack));
        }
#endif

        private void Start()
        {
            UpdateRaySphereRadius();

            direction = (transform.position - CalculateTrackPoint(positionTrack)).normalized;
        }

        private void LateUpdate()
        {
            float inputX = Input.GetAxisRaw("Mouse X") * sensitivity;
            float inputY = Input.GetAxisRaw("Mouse Y") * sensitivity;

            //Angle limit calculations (Limit vertical rotation)
            Vector3 flatYSurface = -transform.forward;
            flatYSurface.y = 0f;
            float signedAngle = Vector3.SignedAngle(flatYSurface, direction, transform.right);

            inputY = Mathf.Clamp(inputY + signedAngle, -85f, 85f) - signedAngle; //General limit (to prevent flip)

            //Rotate direction
            direction = Quaternion.AngleAxis(inputX, Vector3.up) * Quaternion.AngleAxis(inputY, transform.right) * direction;

            //Clamp direction depend on Target (Method commented below), Local limit
            if(isLocked)
                direction = VectorExtensions.ClampAngleAxis(direction, -positionTrack.target.up, verticalLimit.x, verticalLimit.y);

            //Camera zoom in and out
            float currentDistance = 0f;
            if(isZoom)
                currentDistance = zoomDistance;
            else
            {
                distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel") * 5f, distanceClamp.x, distanceClamp.y);
                currentDistance = distance;
            }

            //Set camera position
            Vector3 localDirection = transform.position - CalculateTrackPoint(positionTrack);

            //Collision detection
            if(Physics.SphereCast(CalculateTrackPoint(positionTrack), raySphereRadius, direction, out RaycastHit hitInfo, distance, collisionLayers))
                currentDistance = hitInfo.distance;

            Vector3 slerpDirection = Vector3.Slerp(localDirection, direction * currentDistance, Time.deltaTime * speed);

            transform.position = CalculateTrackPoint(positionTrack) + slerpDirection;

            //Camera rotation
            transform.LookAt(CalculateTrackPoint(lookTrack));
        }

        public void ZoomIn()
        {
            isZoom = true;
        }

        public void ZoomOut()
        {
            isZoom = false;
        }

        public void SetLockedState(bool state)
        {
            isLocked = state;
        }

        //Offsetted track point calculation
        private Vector3 CalculateTrackPoint(TrackInfo t)
        {
            return t.target.position + t.globalOffset + t.target.TransformVector(t.localOffset);
        }

        //Updates ray sphere radius (update the radius if Camera's near clip plane, field of view or aspect is changed)
        private void UpdateRaySphereRadius()
        {
            Vector3 forwardL = transform.forward * cam.nearClipPlane;

            float cameraHeight = Mathf.Tan(cam.fieldOfView * Mathf.Deg2Rad * 0.5f);
            Vector3 upL = transform.up * Mathf.Tan(cameraHeight) * cam.nearClipPlane;

            Vector3 rightL = transform.right * cameraHeight * cam.aspect * cam.nearClipPlane;

            raySphereRadius = (forwardL + upL + rightL).magnitude * 0.5f;
        }

        /*
        /// <summary>
        /// Clamps the vector with angle in relation to axis
        /// </summary>
        /// <param name="vector">The vector that will clamped</param>
        /// <param name="normal">Axis vector, that rotates other vector around</param>
        /// <param name="minAngle">Minimum angle</param>
        /// <param name="maxAngle">Maximum angle</param>
        /// <returns></returns>
        public static Vector3 ClampAngleAxis(Vector3 vector, Vector3 normal, float minAngle, float maxAngle)
        {
            Vector3 right = Vector3.Cross(normal, vector);
            Vector3 forward = Vector3.Cross(right, normal);
            float currentAngle = Vector3.SignedAngle(forward, vector, right);
            float clampedAngle = Mathf.Clamp(currentAngle, minAngle, maxAngle);

            return Quaternion.AngleAxis(clampedAngle - currentAngle, right) * vector;
        }*/
    }
}