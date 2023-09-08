using UnityEngine;

namespace BIK.SemiActive
{
    /// <summary>
    /// IK solver for stepping algorithm
    /// </summary>
    public class SemiTargetIKSolver : IKSolverBase
    {
        [SerializeField] private Transform target; //IK target
        [SerializeField] private float moveDistance; //Stepping distance

        private Vector3 oldPos; //Old world position
        private float lerp; //Lerp value, used in stepping

        private bool rayOverriden = false; //Is ray direction overrided

        public Vector3 rayDir { get; private set; } //Overrided ray direction
        public Vector3 hitNormal { get; private set; } //Ray hit surface normal

        private const float MOVE_UP_LENGTH = 0.4f; //Move up length while stepping
        private const float IDLE_DIST = 0.7f; //Default distance IK must maintain
        private const float LENGTH_OFFSET = 0f; //IK target offset added to the rayLength

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(rayPoint.position, rayPoint.position - rayPoint.up * rayLength);
        }
#endif

        private void Start()
        {
            CheckPos(true);
            lerp = 1f; //Instantly place target to the worldPlacePoint
        }

        private void Update()
        {
            PlaceTarget(worldPlacePoint);
        }

        private void FixedUpdate()
        {
            CheckPos();
        }

        /// <summary>
        /// Use dir as rayDirection insted of -rayPoint.up
        /// </summary>
        /// <param name="dir">Global direction of ray</param>
        public void OverrideRayDirection(Vector3 dir)
        {
            rayDir = dir;

            if (!rayOverriden)
                rayOverriden = true;
        }

        /// <summary>
        /// Disable ray override
        /// </summary>
        public void ResetRayDirection()
        {
            if (rayOverriden)
                rayOverriden = false;
        }

        /// <summary>
        /// Checks if we need to step or not
        /// </summary>
        /// <param name="force">Forces step no matter what</param>
        private void CheckPos(bool force = false)
        {
            Vector3 rayDirection = -rayPoint.up;
            float idleMultiplier = IDLE_DIST;
            if (rayOverriden)
            {
                rayDirection = rayDir;
                idleMultiplier = 1f;
            }

            float offsetRayLength = rayLength + LENGTH_OFFSET;

            RaycastHit hitInfo;
            if (Physics.SphereCast(rayPoint.position, rayRadius, rayDirection, out hitInfo, offsetRayLength, LayerMask.GetMask("Terrain"))
                && Vector3.Dot(transform.right, hitInfo.normal) >= 0.7f) //Cos(45 degrees)
            {
                hitNormal = hitInfo.normal;
                if (force || Vector3.Distance(worldPlacePoint, hitInfo.point) >= moveDistance)
                {
                    StepTo(hitInfo.point);
                }
                else if (isMoving || !onGround)
                {
                    worldPlacePoint = hitInfo.point; //Update world place point if we are still stepping or we are on air
                }
                //else
                //    worldPlacePoint = hitInfo.point;
                onGround = true;
            }
            else if (onGround)
            {
                StepTo(rayPoint.position + rayDirection * offsetRayLength * idleMultiplier);
                onGround = false;
            }
            else
            {
                worldPlacePoint = rayPoint.position + rayDirection * offsetRayLength * idleMultiplier;
            }
            Debug.DrawLine(rayPoint.position, worldPlacePoint, Color.green);
        }

        /// <summary>
        /// Place IK target to the worldPos
        /// </summary>
        /// <param name="worldPos">Target goal world position</param>
        private void PlaceTarget(Vector3 worldPos)
        {
            target.position = Vector3.Lerp(oldPos, worldPos, lerp) + hitNormal * MOVE_UP_LENGTH * Mathf.Sin(lerp * Mathf.PI);

            if (isMoving)
            {
                lerp = Mathf.MoveTowards(lerp, 1f, Time.deltaTime * moveSpeed);
                if (lerp >= 1f)
                {
                    lerp = 1f;
                    isMoving = false;
                }
            }
        }

        /// <summary>
        /// Begins stepping to p world position
        /// </summary>
        /// <param name="p">World position</param>
        private void StepTo(Vector3 p)
        {
            oldPos = worldPlacePoint;
            worldPlacePoint = p;
            isMoving = true;
            lerp = 0;
        }
    }
}