using UnityEngine;

namespace HandWar.SemiActive
{
    public class SemiTargetIKSolver : IKSolverBase
    {
        [SerializeField] private Transform rayPoint;
        [SerializeField] private float rayLength = 3f;
        [SerializeField] private Transform target;
        [SerializeField] private float moveDistance;
        [SerializeField] private float moveSpeed = 2f;
        [SerializeField] private float rayRadius = 0.2f;

        private Vector3 oldPos;
        public Vector3 worldPlacePoint { get; private set; }
        private float lerp;

        private bool rayOverriden = false;

        public Vector3 rayDir { get; private set; }
        public Vector3 hitNormal { get; private set; }

        private const float MOVE_UP_LENGTH = 0.4f;
        private const float IDLE_DIST = 0.7f;
        private const float LENGTH_OFFSET = 0f;

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

        public void OverrideRayDirection(Vector3 dir)
        {
            rayDir = dir;

            if (!rayOverriden)
                rayOverriden = true;
        }

        public void ResetRayDirection()
        {
            if (rayOverriden)
                rayOverriden = false;
        }

        public void ChangeRayLength(float length)
        {
            //rayLength = Mathf.Max(length + rayPoint.localPosition.y - 0.7f, 0f);
            rayLength = length + LENGTH_OFFSET; //a little offset

            //Prevent unnecessary calculation to gain performance
            if (length > 0.1f != enabled)
                enabled = length > 0.1f;
        }

        public float GetRawLength()
        {
            return Mathf.Max(rayLength - LENGTH_OFFSET, 0f);
        }

        private void CheckPos(bool force = false)
        {
            Vector3 rayDirection = -rayPoint.up;
            float idleMultiplier = IDLE_DIST;
            if (rayOverriden)
            {
                rayDirection = rayDir;
                idleMultiplier = 1f;
            }

            RaycastHit hitInfo;
            if (Physics.SphereCast(rayPoint.position, rayRadius, rayDirection, out hitInfo, rayLength, LayerMask.GetMask("Terrain"))
                && Vector3.Dot(transform.right, hitInfo.normal) >= 0.7f) //Cos(45 degrees)
            {
                hitNormal = hitInfo.normal;
                if (force || Vector3.Distance(worldPlacePoint, hitInfo.point) >= moveDistance)
                {
                    ReplaceWorldPlace(hitInfo.point);
                }
                else if (isMoving || !onGround)
                {
                    worldPlacePoint = hitInfo.point;
                }
                //else
                //    worldPlacePoint = hitInfo.point;
                onGround = true;
            }
            else if (onGround)
            {
                ReplaceWorldPlace(rayPoint.position + rayDirection * rayLength * idleMultiplier);
                onGround = false;
            }
            else
            {
                worldPlacePoint = rayPoint.position + rayDirection * rayLength * idleMultiplier;
            }
            Debug.DrawLine(rayPoint.position, worldPlacePoint, Color.green);
        }

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

        private void ReplaceWorldPlace(Vector3 p)
        {
            oldPos = worldPlacePoint;
            worldPlacePoint = p;
            isMoving = true;
            lerp = 0;
        }
    }
}