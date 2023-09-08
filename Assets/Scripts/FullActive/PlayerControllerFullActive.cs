using UnityEngine;
using UnityEngine.InputSystem;
using Extension;

namespace BIK.FullActive
{
    [RequireComponent(typeof(Rigidbody))]
    public class PlayerControllerFullActive : MonoBehaviour
    {
        //Force multipliers
        [SerializeField] private float jumpForceMultiplier = 60f;

        [SerializeField] private float moveRotationalAngle = 60f;
        [SerializeField] private float rotateRotationalAngle = 60f;

        //Lerp multipliers
        [SerializeField, Tooltip("Rotate degree per second")] private float torsoRotateSpeed = 180f;
        [SerializeField] private float torsoRotateSpring = 15000;
        [SerializeField] private float torsoRotateDamper = 150;

        //Aim range (low, high)
        [SerializeField] private Vector2 aimXRange;

        //All fingers
        [SerializeField] private FullIKHolder fingers;

        //Passive body
        [SerializeField] private Transform bodyPassive;

        //Active body, camera and input class
        private Rigidbody body;
        private Transform cam;
        private PlayerInputAction playerInputAction;
        private ConfigurableJoint torsoJoint;
        private Quaternion torsoSpace;

        //Input value converted to Vector3
        private Vector3 inputVector;

        //Jump values
        private const float JUMP_COOLDOWN = 0.3f;
        private float lastJump = 0f;
        private bool doubleJump = false;

        //Jump indicator objects
        [SerializeField, Space(10)] private GameObject jumpRedLed;
        [SerializeField] private GameObject jumpBlueLed;

        //Frequently used variables
        private bool onGround = false;
        private bool canMove = false;
        private bool isAiming = false;
        private bool palmOnGround = false;
        private int activeFingers = 0;

        //Aim finger index
        private FullIKInfo selectedAimFinger = null;

        private const float MAX_ROT_DIFF = 5f; //Maximum rotation range of the passive body from active body

        private const float HOVER_HEIGHT = 1.5f; //Default hover height

        private const float SELECTED_FINGER_HOVER = 70f; //When aiming, hover the hand so it will balance

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            //Draw aimXRange visuals
            const float DISTANCE = 2f;
            const float RESOLUTION = 2f;
            float steps = 1f / RESOLUTION;

            Gizmos.color = Color.blue;
            for(float i = aimXRange.x; i < aimXRange.y; i += steps)
            {
                Gizmos.DrawLine(transform.position + Quaternion.AngleAxis(i, -transform.right) * transform.forward * DISTANCE,
                    transform.position + Quaternion.AngleAxis(i + steps, -transform.right) * transform.forward * DISTANCE);
            }
            Gizmos.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(aimXRange.x, -transform.right) * transform.forward * DISTANCE);
            Gizmos.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(aimXRange.y, -transform.right) * transform.forward * DISTANCE);
        }
    #endif

        private void Awake()
        {
            //Set all components
            body = GetComponent<Rigidbody>();
            torsoJoint = GetComponent<ConfigurableJoint>();

            //Set inputs
            playerInputAction = new PlayerInputAction();
            playerInputAction.Player.Enable();
            playerInputAction.Player.Jump.performed += ctx => Jump();

            playerInputAction.Player.Aim.performed += ctx => AimFinger();
            playerInputAction.Player.Aim.canceled += ctx => ReleaseFinger();
            playerInputAction.Player.Fire.performed += ctx => FireFinger();
        }

        private void Start()
        {
            //Set singleton components
            cam = CameraController.instance.transform;

            torsoSpace = transform.rotation;

            //Changing center of mass helps to balance
            body.centerOfMass = transform.position + transform.forward * 0.5f;
            //body.centerOfMass = transform.InverseTransformPoint(fingers.CalculateCenterOfMass());

            fingers.SetIdealHeight(HOVER_HEIGHT);
        }

        private void Update()
        {
            //Get active finger count
            activeFingers = fingers.GetAvailableIKsCount();

            //Get movement inputs
            Vector2 rawInput = playerInputAction.Player.Movement.ReadValue<Vector2>();
            inputVector = new Vector3(rawInput.x, 0, rawInput.y);

            if (isAiming)
                AimFinger();

            FullActiveMove(Time.deltaTime, out float moveAngle, out Vector3 moveAxis);

            PassiveRotate();
            FullActiveRotate(Time.deltaTime, out float rotateAngle, out Vector3 rotateAxis);

            if (inputVector.sqrMagnitude > 0f)
            {
                rotateAngle *= 0.3f;
                rotateAxis += moveAxis * 0.2f;
            }

            fingers.AddRotation(moveAngle, moveAxis, rotateAngle, rotateAxis);
        }

        private void FixedUpdate()
        {
            //bool befGround = onGround;
            //Update onGround and canMove value (No need to calculate them in each function)
            onGround = fingers.OnGround() || palmOnGround;
            canMove = fingers.CanMove();

            //If we don't have any fingers than check from palm position
            if(!onGround && activeFingers == 0 && Physics.Raycast(body.position, -transform.up, 1.5f))
            {
                //Override onGround value
                onGround = true;
            }

            //Reset double jump bool (Neccessary to update double jump indicator led)
            if (!doubleJump && onGround && Time.time > JUMP_COOLDOWN + lastJump)
                doubleJump = true;

            SetJumpLeds();

            CameraController.instance.SetLockedState(onGround);

            fingers.SetBodyUp(body.rotation * Vector3.up);

            //If we are in air, rotate with torsoJoint
            TorqueRotate();

            bodyPassive.position = body.position;
            //bodyPassive.rotation = body.rotation;

            fingers.SyncJointsWithIK();

            if (isAiming && onGround)
                Balance();
        }

        #region FINGER METHODS
        private void AimFinger()
        {
            if (selectedAimFinger == null || !selectedAimFinger.IsIKAvailable)
                selectedAimFinger = fingers.GetAvailableIK();

            if(selectedAimFinger != null) //There is a possibility where player has no fingers
                selectedAimFinger.solver.OverrideRayDirection(cam.forward);

            if(!isAiming)
            {
                isAiming = true;
                CameraController.instance.ZoomIn();
            }
        }

        private void ReleaseFinger()
        {
            if (selectedAimFinger != null)
                selectedAimFinger.solver.ResetRayDirection();

            if(isAiming)
            {
                isAiming = false;
                CameraController.instance.ZoomOut();
            }
        }

        private void FireFinger()
        {
            if (selectedAimFinger == null || !isAiming)
                return;

            fingers.FireSegment(selectedAimFinger);

            if (!selectedAimFinger.IsIKAvailable)
            {
                selectedAimFinger.solver.ResetRayDirection();
                selectedAimFinger = null;
            }
        }
        #endregion

        /// <summary>
        /// Balance the hand when aiming
        /// </summary>
        private void Balance()
        {
            if (selectedAimFinger == null)
                return;

            //Calculate points as rigidbody center of mass as origin
            Vector3 offCenter = fingers.CalculateCenterOfMass();
            Vector3 goalCenter = fingers.CalculateGroundedCenterOfMass();

            float magn = Vector3.Distance(goalCenter, offCenter);
            Vector3 forceDir = Vector3.Cross(offCenter - goalCenter, transform.right).normalized;

            float ratio = (float)fingers.GetGroundedIKCount()/fingers.TotalIK;

            body.AddForceAtPosition(forceDir * magn * SELECTED_FINGER_HOVER * ratio, selectedAimFinger.GetActiveRoot.position);
        }

        /// <summary>
        /// Rotate the hand with joint force
        /// </summary>
        private void TorqueRotate()
        {
            if (!onGround || activeFingers == 0) //We are falling or we have jumped
            {
                //Aim with keyboard
                float degreeForward = inputVector.z;
                float degreeRight = -inputVector.x;

                Vector3 lookForward = Quaternion.AngleAxis(degreeForward * 30f, transform.right) * transform.forward;
                Vector3 lookUp = Quaternion.AngleAxis(degreeRight * 30f, transform.forward) * transform.up;

                torsoJoint.SetTargetRotation(Quaternion.RotateTowards(body.rotation, Quaternion.LookRotation(lookForward, lookUp), Time.fixedDeltaTime * torsoRotateSpeed), torsoSpace);
                
                if(torsoJoint.slerpDrive.positionSpring <= 0)
                {
                    JointDrive jd = torsoJoint.slerpDrive;
                    jd.positionSpring = torsoRotateSpring;
                    jd.positionDamper = torsoRotateDamper;
                    torsoJoint.slerpDrive = jd;
                }
            }
            else if(torsoJoint.slerpDrive.positionSpring > 0) //We are on ground
            {
                JointDrive jd = torsoJoint.slerpDrive;
                jd.positionSpring = 0;
                jd.positionDamper = 0;
                torsoJoint.slerpDrive = jd;
            }
        }

        /// <summary>
        /// Calculates fingers rotate axis to rotate the hand
        /// </summary>
        /// <param name="delta">Delta time</param>
        /// <param name="rotateAngle">Rotate angle</param>
        /// <param name="rotateAxis">Rotate axis</param>
        private void FullActiveRotate(float delta, out float rotateAngle, out Vector3 rotateAxis)
        {
            Vector3 lookForward;

            if (!onGround || activeFingers == 0) //We are falling or we have jumped
            {
                //Aim with keyboard
                float degreeForward = inputVector.z;
                float degreeRight = -inputVector.x;

                lookForward = Quaternion.AngleAxis(degreeForward * 30f, transform.right) * transform.forward;
            }
            else //We are on ground
            {
                //Calculate surface normal according to finger placements and use it as local up vector
                Vector3 lookUp = fingers.AverageTargetNormalPythagor(transform.position, transform.up).normalized;

                //Calculate forward vector with cross product on right and local up vector
                Vector3 localForward = Vector3.Cross(transform.right, lookUp).normalized;

                Vector3 forwardDirection = cam.forward;

                //Check if we are aiming
                if (isAiming && selectedAimFinger != null)
                {
                    Vector3 diff = selectedAimFinger.GetActiveRoot.position - transform.position;

                    float angle = Vector3.Angle(selectedAimFinger.GetActiveRoot.position - transform.position, cam.forward);
                    forwardDirection = Quaternion.AngleAxis(angle, Vector3.Cross(diff, cam.forward)) * cam.forward;
                }

                //Clamp the vector from local forward to forwardDirection with angle around local up vector
                lookForward = VectorExtensions.ClampAngleAxis(forwardDirection, lookUp, aimXRange.x, aimXRange.y);
            }

            //Debug.DrawRay(transform.position, lookForward, Color.white);
            float dot = Vector3.Dot(transform.right, lookForward);
            rotateAngle = rotateRotationalAngle * dot * delta;
            rotateAxis = Vector3.down;
            //fingers.AddLocalRotation(rotateFullSpeed * dot * delta, Vector3.down);
        }

        /// <summary>
        /// Rotates the passive body to where the camera looks
        /// </summary>
        private void PassiveRotate()
        {
            Vector3 lookUp;
            Vector3 lookForward;

            if (!onGround || activeFingers == 0) //We are falling or we have jumped
            {
                //Aim with keyboard
                float degreeForward = inputVector.z;
                float degreeRight = -inputVector.x;

                lookForward = Quaternion.AngleAxis(degreeForward, bodyPassive.right) * bodyPassive.forward;
                lookUp = Quaternion.AngleAxis(degreeRight, bodyPassive.forward) * bodyPassive.up;
            }
            else //We are on ground
            {
                //Calculate surface normal according to finger placements and use it as local up vector
                lookUp = cam.up - cam.forward * 0.15f;

                //Clamp the vector from local forward to forwardDirection with angle around local up vector
                lookForward = Vector3.Cross(transform.right, lookUp);
            }

            //Clamp calculated rotation for slow rotation movement
            Quaternion clampedRot = Quaternion.RotateTowards(body.rotation, Quaternion.LookRotation(lookForward, lookUp), MAX_ROT_DIFF);

            //Use RotateTowards to smoothly rotate passive body
            Quaternion finalRot = Quaternion.RotateTowards(bodyPassive.rotation, clampedRot, Time.deltaTime * torsoRotateSpeed);
            bodyPassive.rotation = finalRot;
        }

        /// <summary>
        /// Calculates move axis and angle to move the hand
        /// </summary>
        /// <param name="delta">Delta time</param>
        /// <param name="moveAngle">Move angle</param>
        /// <param name="moveAxis">Move axis</param>
        private void FullActiveMove(float delta, out float moveAngle, out Vector3 moveAxis)
        {
            Vector3 inputLocal = InputToLocal(inputVector);

            Vector3 globalRotateAxis = Vector3.Cross(transform.up, inputLocal);
            //Debug.DrawRay(transform.position, globalRotateAxis * 3f, Color.red);
            moveAngle = moveRotationalAngle * delta;
            moveAxis = globalRotateAxis;
        }

        /// <summary>
        /// Adds instant up vector force to the active body
        /// </summary>
        private void Jump()
        {
            if(onGround && Time.time - lastJump > JUMP_COOLDOWN)
            {
                body.AddForce(transform.up * jumpForceMultiplier, ForceMode.Impulse);
                doubleJump = true;
                lastJump = Time.time;
            }
            else if(doubleJump)
            {
                body.AddForce(transform.up * jumpForceMultiplier, ForceMode.Impulse);
                doubleJump = false;
            }
        }

        /// <summary>
        /// Sets the jump leds on the circuit board
        /// </summary>
        private void SetJumpLeds()
        {
            if (jumpBlueLed.activeSelf != onGround)
                jumpBlueLed.SetActive(onGround);

            if (jumpRedLed.activeSelf != doubleJump)
                jumpRedLed.SetActive(doubleJump);
        }

        /// <summary>
        /// Converts input global vector to local input vector
        /// </summary>
        /// <param name="inputV">Input vector</param>
        /// <returns></returns>
        private Vector3 InputToLocal(Vector3 inputV)
        {
            Vector3 flatCamForward = cam.forward;
            flatCamForward.y = 0f;
            float angle = Vector3.SignedAngle(Vector3.forward, flatCamForward, Vector3.up);
            Vector3 inputLocal = Quaternion.AngleAxis(angle, Vector3.up) * inputV;
            return Quaternion.AngleAxis(transform.localEulerAngles.x, transform.right) * inputLocal;
        }

        private void OnCollisionEnter(Collision collision)
        {
            palmOnGround = true;
        }

        private void OnCollisionExit(Collision collision)
        {
            palmOnGround = false;
        }
    }
}