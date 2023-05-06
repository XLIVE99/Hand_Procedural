using UnityEngine;
using Extension;

namespace HandWar
{
    [RequireComponent(typeof(Rigidbody))]
    public class PlayerController : MonoBehaviour
    {
        //Force multipliers
        [SerializeField] private float moveForceMultiplier = 10f;
        //[SerializeField] private float hoverForceMultiplier = 10f; //Hover is done in SyncMovement
        [SerializeField] private float torqueForwardForceMultiplier = 10f, torqueUpForceMultiplier = 4f;
        [SerializeField] private float jumpForceMultiplier = 60f;

        //Lerp multipliers
        [SerializeField, Space(10)] private float moveXZLerp = 3f;
        [SerializeField] private float moveYLerp = 10f;
        [SerializeField] private float airLerp = 10f;
        [SerializeField, Tooltip("Rotate degree per second")] private float rotateSpeed = 180f;

        //Aim range (low, high)
        [SerializeField] private Vector2 aimXRange;

        //All fingers
        [SerializeField] private IKHolder fingers;

        //PIDs
        [SerializeField] private PIDController pidMove, pidTorque;

        //Passive body
        [SerializeField] private Transform bodyPassive;

        //Active body and camera
        private Rigidbody body;
        private Transform cam;

        //PID values
        private PIDStore<Vector3> pidStoreMove;
        private PIDStore<Vector3> pidStoreTorque;

        //Input value converted to Vector3
        private Vector3 inputVector;

        //The passive body default position (to clamp distance from the active body)
        private Vector3 passivePos;

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

        //Aim finger index
        private IKInfo selectedAimFinger = null;

        private const float MIN_POS_DIFF = 1.0f; //Minimum distance between passive and active body
        private const float MAX_POS_DIFF = 2f; //Maximum movement range of the passive body

        private const float MAX_ROT_DIFF = 10f; //Maximum rotation range of the passive body from active body

        private const float HOVER_HEIGHT = 2f; //Default hover height

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
            //Create new PID store class for PID calculations
            pidStoreMove = new PIDStore<Vector3>();
            pidStoreTorque = new PIDStore<Vector3>();

            //Set all components
            body = GetComponent<Rigidbody>();
        }

        private void Start()
        {
            //Set singleton components
            cam = CameraController.instance.transform;

            //Set center of mass to finger's average position
            body.centerOfMass = fingers.CenterOfMass() - body.position;
        }

        private void Update()
        {
            //Get movement inputs
            inputVector = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical"));

            if (Input.GetKeyDown(KeyCode.Space))
                Jump();

            if (Input.GetMouseButton(1))
            {
                AimFinger();
                if (Input.GetMouseButtonDown(0))
                    FireFinger();

                if (!isAiming) //Prevent repeated assigning
                    isAiming = true;
            }
            if(Input.GetMouseButtonDown(1))
            {
                CameraController.instance.ZoomIn();
            }
            else if (Input.GetMouseButtonUp(1))
            {
                CameraController.instance.ZoomOut();

                ReleaseFinger();

                isAiming = false;
            }

            //Passive body movement
            PassiveMove();

            PassiveRotate();

            Hover();
        }

        private void FixedUpdate()
        {
            //Update onGround and canMove value (No need to calculate them in each function)
            onGround = fingers.OnGround();
            canMove = fingers.CanMove();

            //Reset double jump bool (Neccessary to update double jump indicator led)
            if (onGround && Time.time > JUMP_COOLDOWN + lastJump)
                doubleJump = true;

            SetJumpLeds();

            //Sync active body with passive body
            //Aim();
            SyncRotation();

            //Hover();

            //Movement();
            SyncMovement();

            fingers.SyncJointsWithIK();
        }

        private void AimFinger()
        {
            if (selectedAimFinger == null || !selectedAimFinger.IsIKAvailable)
                selectedAimFinger = fingers.GetAvailableIK();

            if(selectedAimFinger != null) //There is a possibility where player has no fingers
                selectedAimFinger.solver.OverrideRayDirection(cam.forward);
        }

        private void ReleaseFinger()
        {
            if (selectedAimFinger != null)
                selectedAimFinger.solver.ResetRayDirection();
        }

        private void FireFinger()
        {
            if (selectedAimFinger == null)
                return;

            fingers.FireSegment(selectedAimFinger);

            if (!selectedAimFinger.IsIKAvailable)
            {
                selectedAimFinger.solver.ResetRayDirection();
                selectedAimFinger = null;
            }
        }

        /// <summary>
        /// Syncs the active body rotation with the passive body rotation
        /// </summary>
        private void SyncRotation()
        {
            //Since body = transform, we can use transform
            //Calculate up and forward torque to match active body with passive body
            Vector3 palmUp = Vector3.Cross(transform.up, bodyPassive.up) * torqueUpForceMultiplier;
            Vector3 palmForward = Vector3.Cross(transform.forward, bodyPassive.forward) * torqueForwardForceMultiplier;

            //Use PID to smoothly rotate active body rotation
            Vector3 palmTorque = pidTorque.UpdatePID(Time.fixedDeltaTime, body.angularVelocity, palmUp + palmForward, pidStoreTorque);

            //Finally, set torque
            body.AddTorque(palmTorque);
        }

        /// <summary>
        /// Rotates the passive body to where the camera looks
        /// </summary>
        private void PassiveRotate()
        {
            Vector3 lookUp;
            Vector3 lookForward;

            if (!onGround) //We are falling or we have jumped
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
                lookUp = fingers.AverageTargetNormalPythagor(bodyPassive.position, bodyPassive.up).normalized;

                //Calculate forward vector with cross product on right and local up vector
                Vector3 localForward = Vector3.Cross(bodyPassive.right, lookUp).normalized;

                Vector3 forwardDirection = cam.forward;
                //Check if we are aiming
                /*if(isAiming && selectedAimFinger != null)
                {
                    Vector3 diff = selectedAimFinger.rootActive.position - transform.position;
                    Vector3 diffProjected = Vector3.ProjectOnPlane(diff, lookUp);
                    Vector3 camProjected = Vector3.ProjectOnPlane(cam.forward, lookUp);
                    float angle = Vector3.Angle(selectedAimFinger.rootActive.position - transform.position, cam.forward);
                    forwardDirection = Quaternion.AngleAxis(angle, Vector3.Cross(diff, cam.forward)) * cam.forward;
                }*/

                //Clamp the vector from local forward to forwardDirection with angle around local up vector
                lookForward = VectorExtensions.ClampAngleAxis(localForward, forwardDirection, lookUp, aimXRange.x, aimXRange.y);
                Debug.DrawRay(transform.position, lookForward, Color.red);
                Debug.DrawRay(transform.position, lookUp, Color.green);
                Debug.DrawLine(transform.position, cam.position, Color.blue);
            }

            //Clamp calculated rotation for slow rotation movement
            Quaternion clampedRot = Quaternion.RotateTowards(body.rotation, Quaternion.LookRotation(lookForward, lookUp), MAX_ROT_DIFF);

            //Use RotateTowards to smoothly rotate passive body
            Quaternion finalRot = Quaternion.RotateTowards(bodyPassive.rotation, clampedRot, Time.deltaTime * rotateSpeed);
            bodyPassive.rotation = finalRot;
        }

        /// <summary>
        /// Hovers the passive body
        /// </summary>
        private void Hover()
        {
            //Check if we are on ground
            if (!onGround)
                return;

            //Calculate default hover height
            float goalHover = fingers.AverageTargetPosition().y + HOVER_HEIGHT;

            //Check if default hover height is reachable
            goalHover = Mathf.Min(goalHover, fingers.MaxHeight(bodyPassive.position, bodyPassive.up));

            //Hover passive body to the goal Y position. Lerp will help passive body to adjust as fast as the difference getting larger.
            Vector3 hoverPos = new Vector3(bodyPassive.position.x,
                Mathf.Lerp(bodyPassive.position.y, goalHover, Time.deltaTime * 20f),
                bodyPassive.position.z);
            bodyPassive.position = hoverPos;
        }

        /// <summary>
        /// Syncs the active body position with the passive body position
        /// </summary>
        private void SyncMovement()
        {
            //Check if we can move and are on ground
            if (!canMove || !onGround)
            {
                return;
            }

            //Remove body.velocity.y axis from the velocity to prevent gravity interference on PID
            //If body.velocity is used, hand behaviour will be a little bit floaty but won't cause a big difference
            //So either body.velocity or velocity2D can be used
            Vector3 velocity2D = transform.InverseTransformDirection(body.velocity);
            velocity2D.y = 0f;
            velocity2D = transform.TransformDirection(velocity2D);

            //Set velocity to difference between passive body and active body then use PID to smooth movement force
            Vector3 moveForce = pidMove.UpdatePID(Time.fixedDeltaTime, velocity2D, (bodyPassive.position - body.position) * moveForceMultiplier, pidStoreMove);
            body.AddForce(moveForce);
        }

        /// <summary>
        /// Moves the passive body with a movement range, that way the passive body can't go far from the active body
        /// </summary>
        private void PassiveMove()
        {
            //Check if we can move and are on ground
            if (!canMove || !onGround)
            {
                //Slowly syncs passive body position to active body position if pasive body is too far away

                //Teleport passive body if it is far away too much
                Vector3 diff = bodyPassive.position - body.position;
                if (diff.magnitude > MAX_POS_DIFF)
                    bodyPassive.position = body.position + (diff.normalized * MAX_POS_DIFF);

                //Lerp passive body slowly to active body
                bodyPassive.position = Vector3.Lerp(bodyPassive.position, body.position, Time.deltaTime * airLerp);
                return;
            }

            //Rotate input vector to where the camera looks
            Vector3 inputLocal = InputToLocal(inputVector);

            //Set passive pos to active body position
            //Can be used either of passivePos calculations
            //passivePos = Vector3.Lerp(passivePos, body.position, (body.position - passivePos).magnitude / MAX_POS_DIFF);
            passivePos = body.position;

            //Separating input vector into axes
            Vector3 movementLocalXZ = Vector3.ProjectOnPlane(inputLocal, bodyPassive.up);
            //Calculate from difference (Because jump done with addForce to the active body)
            Vector3 movementLocalY = Vector3.Project(body.position - bodyPassive.position, bodyPassive.up);

            //Calculate final pos with constant distance to prevent passive body to go far away from active body
            Vector3 finalPos = passivePos + movementLocalXZ * MAX_POS_DIFF;

            //Lerp with different values
            Vector3 lerpFinal = bodyPassive.position;
            if(Vector3.Distance(bodyPassive.position, finalPos) > MIN_POS_DIFF)
                lerpFinal = Vector3.Lerp(bodyPassive.position, finalPos, Time.deltaTime * moveXZLerp);
            lerpFinal = Vector3.Lerp(lerpFinal, lerpFinal + movementLocalY, Time.deltaTime * moveYLerp);

            //Move with lerp to move passive body faster as distance between passive body and final position gets bigger
            bodyPassive.position = lerpFinal;
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
    }
}