using UnityEngine;

namespace BIK.FullActive
{
    /// <summary>
    /// IK solver for stepping algorithm
    /// </summary>
    public class FullTargetIKSolver : IKSolverBase
    {
        //NEEDS REWORK, TOO COMPLICATED
        [SerializeField] private Transform secondRayPoint; //Rotating ray point to grasp ground
        [SerializeField] private Transform target; //IK target
        [SerializeField] private float moveDot = 1.5f; //Step dot condition. If added dot value bigger than this, step
        [SerializeField] private float targetMoveSpeed = 5f; //IK target move speed
        [SerializeField] private float stepAngle = 20f; //Maximum step angle offset
        [SerializeField, Tooltip("In degrees/s")] private float relocateSpeed = 25f; //If finger in air, rotate speed of second ray looking at finger tip
        [SerializeField, Tooltip("In degrees/s")] private float resetSpeed = 180f; //If second ray doesn't hit, rotate speed of returning to main ray rotation
        [SerializeField, Tooltip("In degrees/s")] private float stepReturnSpeed = 90f; //Step returning speed
        [SerializeField] private FullIKInfo ikInfo; //Developer Note: Find a way to discard this variable, depending on this class makes me uneasy
        [SerializeField] private PhysicMaterial lowFriction, highFriction; //Friction materials used in stepping
        [SerializeField] private Collider tipCollider; //Collider which material will change while stepping
        private float tipMass;

        private Quaternion localOffset; //Raypoint rotation offset, used to overshoot second ray rotation while resetting second ray

        private Vector3 bodyUp; //Whole body up vector, used to calculate idealHeight
        private float idealHeight; //Ideal height to stand the body up

        private Quaternion rayLocalRot; //Raypoint local rotation
        private Quaternion rayLocalRotOffset; //Raypoint local rotation additional offset
        private Quaternion GetRayLocalRotation => rayLocalRot * rayLocalRotOffset; //Calculated raypoint local rotation

        public bool rayOverriden { get; private set; } = false; //Is ray direction overrided

        public Vector3 rayDir { get; private set; } //Overrided ray direction
        public Vector3 hitNormal { get; private set; } //Raycast hit surface normal

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

        /// <summary>
        /// Enable ik root object
        /// </summary>
        [ContextMenu("Enable roots")]
        private void EnableRoots()
        {
            ikInfo.GetPassiveRoot.gameObject.SetActive(true);
            ikInfo.GetActiveRoot.gameObject.SetActive(true);
            enabled = true;
        }

        /// <summary>
        /// Disable ik root object
        /// </summary>
        [ContextMenu("Disable roots")]
        private void DisableRoots()
        {
            ikInfo.GetPassiveRoot.gameObject.SetActive(false);
            ikInfo.GetActiveRoot.gameObject.SetActive(false);
            enabled = false;
        }
#endif

        private void Awake()
        {
            if (tipCollider.TryGetComponent(out Rigidbody body))
                tipMass = body.mass;
            else
                Debug.LogWarning("tip collider (" + tipCollider.name + ") doesn't have any Rigidbody. This may cause stutter in movement", tipCollider);
        }

        private void Start()
        {
            worldPlacePoint = target.position;
            CheckPos(Time.deltaTime);
            rayLocalRot = rayPoint.localRotation;
            secondRayPoint.rotation = rayPoint.rotation;
            localOffset = Quaternion.identity;
            rayLocalRotOffset = Quaternion.identity;
        }

        private void FixedUpdate()
        {
            CheckPos(Time.fixedDeltaTime);
        }

        /// <summary>
        /// Change active tip collider
        /// </summary>
        /// <param name="col">collider</param>
        public void ChangeTipCollider(Collider col)
        {
            if(col == null)
            {
                Debug.LogWarning("Tip collider setted to null. This solver will set itself as disabled", this);
                enabled = false;
                tipCollider = null;
                return;
            }

            tipCollider = col;
            if (tipCollider.TryGetComponent(out Rigidbody body))
                tipMass = body.mass;
            else
                Debug.LogWarning("tip collider (" + tipCollider.name + ") doesn't have any Rigidbody. This may cause stutter in movement", tipCollider);
        }

        /// <summary>
        /// Set body up vector
        /// </summary>
        /// <param name="upDir">up vector</param>
        public void SetBodyUp(Vector3 upDir)
        {
            bodyUp = upDir;
        }

        /// <summary>
        /// Set ideal height
        /// </summary>
        /// <param name="idealH"></param>
        public void SetIdealHeight(float idealH)
        {
            idealHeight = idealH;
        }

        /// <summary>
        /// Calculates additional small offset for raypoint local rotation
        /// </summary>
        /// <param name="worldPlace"></param>
        /// <param name="bodyUp"></param>
        public void CalculateRayLocalOffset(Vector3 worldPlace, Vector3 bodyUp)
        {
            Vector3 diff = rayPoint.position - worldPlace;
            float angle = Vector3.SignedAngle(transform.rotation * rayLocalRot * Vector3.up, diff, bodyUp);
            rayLocalRotOffset = Quaternion.AngleAxis(Mathf.Clamp(angle, -15, 15), Vector3.up);
            Debug.DrawRay(rayPoint.position, diff.normalized, Color.yellow);
            Debug.DrawRay(rayPoint.position, transform.rotation * rayLocalRotOffset * Vector3.up, Color.green);
        }

        /// <summary>
        /// Overrides ray direction
        /// </summary>
        /// <param name="dir"></param>
        public void OverrideRayDirection(Vector3 dir)
        {
            rayDir = dir;

            if (!rayOverriden)
                rayOverriden = true;
        }

        /// <summary>
        /// Stops ray override
        /// </summary>
        public void ResetRayDirection()
        {
            if (rayOverriden)
                rayOverriden = false;
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

        /// <summary>
        /// Rotates second ray on global axis
        /// </summary>
        /// <param name="angle">Rotate angle</param>
        /// <param name="axis">Rotate global axis</param>
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

        /// <summary>
        /// Rotates second ray on local axis
        /// </summary>
        /// <param name="angle">Rotate angle</param>
        /// <param name="axis">Rotate local axis</param>
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

        /// <summary>
        /// Rotate second ray local or global (Combined version of global rotate and local rotate)
        /// </summary>
        /// <param name="globalAngle">Global rotate angle</param>
        /// <param name="globalAxis">Global rotate axis</param>
        /// <param name="localAngle">Local rotate angle</param>
        /// <param name="localAxis">Local rotate axis</param>
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

        /// <summary>
        /// Smooth second ray rotation and clamp rotation
        /// </summary>
        /// <param name="finalRot">Desired rotation</param>
        private void SmoothSecondRotation(Quaternion finalRot)
        {
            CalculateActiveLookRotation(secondRayPoint, out Vector3 secondForward, out Vector3 secondUp);

            Quaternion threshold = Quaternion.RotateTowards(Quaternion.LookRotation(secondForward, secondUp), secondRayPoint.rotation, ROTATE_THRESHOLD);
            secondRayPoint.rotation = Quaternion.RotateTowards(threshold, finalRot, MAX_SECOND_ROTATION);
        }

        /// <summary>
        /// Checks IK target position
        /// </summary>
        /// <param name="delta">Delta time</param>
        private void CheckPos(float delta)
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

        /// <summary>
        /// Get ready everything for step up
        /// </summary>
        private void StepUp()
        {
            isMoving = true;
            tipCollider.material = lowFriction;
            tipCollider.GetComponent<Rigidbody>().mass = tipMass * 0.2f;

            CalculateActiveLookRotation(secondRayPoint, out Vector3 forw, out Vector3 up);
            secondRayPoint.rotation = Quaternion.LookRotation(forw, up);

            //Difference between default ray rotation and current ray rotation
            localOffset = Quaternion.Inverse(transform.rotation * GetRayLocalRotation) * rayPoint.rotation;
        }

        /// <summary>
        /// Set everything when step completed
        /// </summary>
        private void StepCompleted()
        {
            isMoving = false;
            tipCollider.material = highFriction;
            tipCollider.GetComponent<Rigidbody>().mass = tipMass;
        }

        /// <summary>
        /// Step algorithm (ground)
        /// </summary>
        /// <param name="delta">Delta time</param>
        /// <returns>Final IK step position</returns>
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

        /// <summary>
        /// Stepping algorithm (air)
        /// </summary>
        /// <param name="delta">Delta time</param>
        /// <returns></returns>
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

        /// <summary>
        /// Calculates forward and up vector which looks at active IK tip point
        /// </summary>
        /// <param name="rayP">Ray point</param>
        /// <param name="forward">Forward vector</param>
        /// <param name="up">Up vector</param>
        private void CalculateActiveLookRotation(Transform rayP, out Vector3 forward, out Vector3 up)
        {
            Vector3 diffToTip = ikInfo.GetActiveTip.position - rayP.position;
            forward = diffToTip.normalized;
            up = Vector3.Cross(forward, transform.right);
        }
    }
}