using UnityEngine;

namespace HandWar.FullActive
{
    public class FullTargetIKSolver : IKSolverBase
    {
        //NEEDS REWORKS
        [SerializeField] private Transform rayPoint;
        [SerializeField] private Transform secondRayPoint;
        [SerializeField] private float rayLength = 3f;
        [SerializeField] private Transform target;
        [SerializeField] private float moveDot = 1.5f;
        [SerializeField] private float targetMoveSpeed = 5f;
        [SerializeField] private float stepAngle = 20f;
        [SerializeField, Tooltip("In degrees/s")] private float relocateSpeed = 25f; //If finger in air, rotate speed of second ray looking at finger tip
        [SerializeField, Tooltip("In degrees/s")] private float resetSpeed = 180f; //If second ray doesn't hit, rotate speed of returning to main ray rotation
        [SerializeField, Tooltip("In degrees/s")] private float stepReturnSpeed = 90f; //Step returning speed
        [SerializeField] private float rayRadius = 0.2f;
        [SerializeField] private FullIKInfo ikInfo; //Developer Note: Find a way to discard this variable, depending on this class makes me uneasy
        [SerializeField] private PhysicMaterial lowFriction, highFriction;
        [SerializeField] private Collider col;

        //private Vector3 oldPos;
        public Vector3 worldPlacePoint { get; private set; }
        private Quaternion localOffset;

        private Vector3 bodyUp;
        private float idealHeight;

        private Quaternion rayLocalRot;
        private Quaternion rayLocalRotOffset;
        private Quaternion GetRayLocalRotation => rayLocalRot * rayLocalRotOffset;

        public bool rayOverriden { get; private set; } = false;

        public Vector3 rayDir { get; private set; }
        public Vector3 hitNormal { get; private set; }

        private const float LENGTH_OFFSET = 0.5f; //Ray length offset
        private const float ROTATE_THRESHOLD = 10f; //Minimum correct angle to active bone tip
        private const float MAX_SECOND_ROTATION = 15f; //Maximum second ray rotation difference (In degrees)

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(rayPoint.position, rayPoint.position + rayPoint.forward * rayLength);

            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(secondRayPoint.position, secondRayPoint.position + secondRayPoint.forward * rayLength);
        }

        [ContextMenu("Enable roots")]
        private void EnableRoots()
        {
            ikInfo.GetPassiveRoot.gameObject.SetActive(true);
            ikInfo.GetActiveRoot.gameObject.SetActive(true);
            enabled = true;
        }

        [ContextMenu("Disable roots")]
        private void DisableRoots()
        {
            ikInfo.GetPassiveRoot.gameObject.SetActive(false);
            ikInfo.GetActiveRoot.gameObject.SetActive(false);
            enabled = false;
        }
#endif

        private void Start()
        {
            worldPlacePoint = target.position;
            CheckPos(Time.deltaTime, true);
            rayLocalRot = rayPoint.localRotation;
            secondRayPoint.rotation = rayPoint.rotation;
            localOffset = Quaternion.identity;
            rayLocalRotOffset = Quaternion.identity;
        }

        private void FixedUpdate()
        {
            CheckPos(Time.fixedDeltaTime);
        }

        public void SetBodyUp(Vector3 upDir)
        {
            bodyUp = upDir;
        }

        public void SetIdealHeight(float idealH)
        {
            idealHeight = idealH;
        }

        public void CalculateRayLocalOffset(Vector3 worldPlace, Vector3 bodyUp)
        {
            Vector3 diff = rayPoint.position - worldPlace;
            float angle = Vector3.SignedAngle(transform.rotation * rayLocalRot * Vector3.up, diff, bodyUp);
            rayLocalRotOffset = Quaternion.AngleAxis(Mathf.Clamp(angle, -15, 15), Vector3.up);
            Debug.DrawRay(rayPoint.position, diff.normalized, Color.yellow);
            Debug.DrawRay(rayPoint.position, transform.rotation * rayLocalRotOffset * Vector3.up, Color.green);
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

        // Called from CollisionDetector.cs
        public void TipCollisionEnter()
        {
            onGround = true;
        }

        // Called from CollisionDetector.cs
        public void TipCollisionExit()
        {
            onGround = false;
        }

        public void RotateSecondRayGlobal(float angle, Vector3 axis)
        {
            if (!isMoving && onGround && !rayOverriden)
            {
                Quaternion result = Quaternion.AngleAxis(angle, axis) * secondRayPoint.rotation;
                SmoothSecondRotation(result);

                if (axis.sqrMagnitude > 0.5f)
                    rayPoint.rotation = transform.rotation * Quaternion.AngleAxis(stepAngle, -axis) * GetRayLocalRotation;
                else
                    rayPoint.localRotation = GetRayLocalRotation;
            }
        }

        public void RotateSecondaryRayLocal(float angle, Vector3 axis)
        {
            if (!isMoving && onGround && !rayOverriden)
            {
                Quaternion result = secondRayPoint.rotation * Quaternion.AngleAxis(angle, axis);
                SmoothSecondRotation(result);

                if(angle > 2f)
                    rayPoint.localRotation = GetRayLocalRotation * Quaternion.AngleAxis(stepAngle, -axis);
            }
        }

        public void RotateRays(float globalAngle, Vector3 globalAxis, float localAngle, Vector3 localAxis)
        {
            if(!isMoving && onGround && !rayOverriden)
            {
                Quaternion result = Quaternion.AngleAxis(globalAngle, globalAxis) * secondRayPoint.rotation * Quaternion.AngleAxis(localAngle, localAxis);
                SmoothSecondRotation(result);

                if (globalAxis.sqrMagnitude > 0.5f || localAngle > 5f)
                    rayPoint.rotation = transform.rotation * Quaternion.AngleAxis(stepAngle, globalAxis) * GetRayLocalRotation * Quaternion.AngleAxis(stepAngle, localAxis);
                else
                    rayPoint.localRotation = GetRayLocalRotation;
            }
        }

        private void SmoothSecondRotation(Quaternion finalRot)
        {
            CalculateActiveLookRotation(secondRayPoint, out Vector3 secondForward, out Vector3 secondUp);

            Quaternion threshold = Quaternion.RotateTowards(Quaternion.LookRotation(secondForward, secondUp), secondRayPoint.rotation, ROTATE_THRESHOLD);
            secondRayPoint.rotation = Quaternion.RotateTowards(threshold, finalRot, MAX_SECOND_ROTATION);
        }

        private void CheckPos(float delta, bool force = false)
        {
            if (isMoving)
            {
                worldPlacePoint = RaycastStep(delta);

                target.position = Vector3.Lerp(target.position, worldPlacePoint, delta * targetMoveSpeed * 2f);
            }
            else
            {
                worldPlacePoint = RayPlacePoint(delta);

                target.position = Vector3.Lerp(target.position, worldPlacePoint, delta * targetMoveSpeed);
            }
        }

        private void StepUp()
        {
            isMoving = true;
            col.material = lowFriction;
            col.GetComponent<Rigidbody>().mass = 0.6f;

            CalculateActiveLookRotation(secondRayPoint, out Vector3 forw, out Vector3 up);
            secondRayPoint.rotation = Quaternion.LookRotation(forw, up);

            //Difference between default ray rotation and current ray rotation
            localOffset = Quaternion.Inverse(transform.rotation * GetRayLocalRotation) * rayPoint.rotation;
        }

        private void StepCompleted()
        {
            isMoving = false;
            col.material = highFriction;
            col.GetComponent<Rigidbody>().mass = 3f;
        }

        private Vector3 RayPlacePoint(float delta)
        {
            Vector3 result = Vector3.zero;

            Vector3 rayDirection = secondRayPoint.forward;
            if (rayOverriden)
                rayDirection = rayDir;

            Physics.SphereCast(secondRayPoint.position, rayRadius, rayDirection, out RaycastHit secondHit, rayLength + LENGTH_OFFSET, LayerMask.GetMask("Terrain"));

            if(rayOverriden) //We are aiming the finger
            {
                if (secondHit.transform != null)
                    return secondRayPoint.position + rayDirection * Mathf.Clamp(secondHit.distance, ikInfo.extentMin, ikInfo.extentMax);
                else
                    return secondRayPoint.position + rayDirection * (rayLength + LENGTH_OFFSET);
            }

            float dotMain = Vector3.Dot(rayPoint.forward, secondRayPoint.forward);
            float dotOffset = Vector3.Dot(transform.rotation * GetRayLocalRotation * Vector3.forward, secondRayPoint.forward);

            //Old step condition
            //if (secondHit.transform != null && dotMain + dotOffset > moveDot && Vector3.Dot(secondHit.normal, secondRayPoint.forward) < -0.2f) //Cos(45 degrees)
            if (dotMain + dotOffset > moveDot) //New step condition
            {
                Vector3 finalPoint = Vector3.zero;
                if(secondHit.transform != null)
                {
                    if(secondHit.distance > ikInfo.extentMax)
                    {
                        StepUp();
                        finalPoint = secondRayPoint.position + secondRayPoint.forward * ikInfo.extentMax;
                    }
                    else
                    {
                        //Calculate ideal height, so we dont always stand with complete straight fingers
                        float idealLength = (Vector3.Project(secondRayPoint.forward, bodyUp) * idealHeight).magnitude;

                        finalPoint = secondHit.point + secondRayPoint.forward * Mathf.Clamp(idealLength - secondHit.distance, -0.2f, 0.2f);
                        if (secondHit.distance < ikInfo.extentMin)
                            finalPoint = secondRayPoint.position + secondRayPoint.forward * ikInfo.extentMin;

                        if (!onGround) //Not on ground, rotate second ray to look at finger tip
                        {
                            Vector3 relocatedTarget = target.position + ikInfo.activeSegments[0].position - ikInfo.passiveSegments[0].position;
                            CalculateActiveLookRotation(secondRayPoint, out Vector3 forw, out Vector3 up);

                            secondRayPoint.rotation = Quaternion.RotateTowards(secondRayPoint.rotation, Quaternion.LookRotation(forw, up), delta * relocateSpeed);

                            if (Vector3.Distance(relocatedTarget, ikInfo.GetActiveTip.position) < 0.2f)
                                onGround = true;
                        }
                    }
                }
                else
                {
                    secondRayPoint.rotation = Quaternion.RotateTowards(secondRayPoint.rotation, transform.rotation * GetRayLocalRotation, delta * resetSpeed);
                    finalPoint = secondRayPoint.position + secondRayPoint.forward * ikInfo.extentMax;
                }

                result = finalPoint;
            }
            else
            {
                if (Vector3.Angle(secondRayPoint.forward, rayPoint.forward) > 1f)
                {
                    StepUp();
                }

                result = secondRayPoint.position + secondRayPoint.forward * ikInfo.extentMax;
            }

            return result;
        }

        private Vector3 RaycastStep(float delta)
        {
            Vector3 result = Vector3.zero;

            Quaternion goalRotation = transform.rotation * GetRayLocalRotation * localOffset;

            secondRayPoint.rotation = Quaternion.RotateTowards(secondRayPoint.rotation, goalRotation, delta * stepReturnSpeed);

            Physics.Raycast(secondRayPoint.position, secondRayPoint.forward, out RaycastHit secondHit, rayLength + LENGTH_OFFSET, LayerMask.GetMask("Terrain"));

            if (secondHit.transform != null && secondHit.distance < ikInfo.extentMax)
            {
                result = secondHit.point - secondRayPoint.forward * 0.5f;
                if (Vector3.Distance(result, secondRayPoint.position) < ikInfo.extentMin)
                    result = secondRayPoint.position + secondRayPoint.forward * ikInfo.extentMin;
            }
            else
            {
                result = secondRayPoint.position + secondRayPoint.forward * rayLength;
            }

            if (Quaternion.Angle(secondRayPoint.rotation, goalRotation) < 5f)
            {
                StepCompleted();
            }

            return result;
        }

        private void CalculateActiveLookRotation(Transform rayP, out Vector3 forward, out Vector3 up)
        {
            Vector3 diffToTip = ikInfo.GetActiveTip.position - rayP.position;
            forward = diffToTip.normalized;
            up = Vector3.Cross(forward, transform.right);
        }
    }
}