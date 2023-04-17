using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PlayerControllerActive : MonoBehaviour
{
    //Force multipliers
    [SerializeField] private float moveForceMultiplier = 10f;
    [SerializeField] private float hoverForceMultiplier = 10f;
    [SerializeField] private float torqueForwardForceMultiplier = 10f, torqueUpForceMultiplier = 4f;

    //Aim range (low, high)
    [SerializeField] private Vector2 aimXRange;

    //All fingers
    [SerializeField] private IKHolder[] fingers;

    //PIDs
    [SerializeField] private PIDController pidMove, pidTorque, pidHover;

    //Passive body
    [SerializeField] private Transform bodyPassive;

    //Active body and camera
    private Rigidbody body;
    private Transform cam;

    //PID values
    private PIDStore<Vector3> pidStoreMove;
    private PIDStore<Vector3> pidStoreTorque;
    private PIDStore<float> pidStoreHover;

    //Input value converted to Vector3
    private Vector3 inputVector;

    //The passive body default position (to clamp distance from the active body)
    private Vector3 passivePos;
    private const float MAX_POS_DIFF = 2f; //Maximum movement range of the passive body

    private const float HOVER_HEIGHT = 2f; //Default hover height

    //Can be converted to class in order to write methods inside of IKHolder
    [System.Serializable]
    public struct IKHolder
    {
#if UNITY_EDITOR
        public string ikName; //IK name to easily see it on the inspector
#endif
        public Transform root; //IK passive body root
        public Transform rootActive; //IK active body root (joint root)
        public TargetIKSolver solver; //IK solver
        public float extentMax; //Maximum extent of the finger
        public float extentMin; //Minimum extent of the finger (not used)

        [HideInInspector]
        public Quaternion[] cachedJointSpace; //Cached joint spaces (used to rotates the joints)
    }

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

    //Easy way to calculate max extent
    [ContextMenu("Fingers length")]
    private void CalculateFingerLengths()
    {
        foreach(IKHolder finger in fingers)
        {
            float dist = recursiveLength(finger.root);
            Debug.Log(finger.root.name + " length: " + dist);
        }
    }

    private float recursiveLength(Transform parent)
    {
        if (parent.childCount <= 0)
            return 0f;
        else
        {
            return Vector3.Distance(parent.position, parent.GetChild(0).position) + recursiveLength(parent.GetChild(0));
        }
    }
#endif

    private void Awake()
    {
        //Create new PID store class for PID calculations
        pidStoreMove = new PIDStore<Vector3>();
        pidStoreTorque = new PIDStore<Vector3>();
        pidStoreHover = new PIDStore<float>();

        //Set all components
        body = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        //Set singleton components
        cam = CameraController.instance.transform;

        //Modify rigidbody's center of mass
        Vector3 centerOfMass = Vector3.zero;

        //Calculate mid point based on all finger root's positions
        foreach(IKHolder finger in fingers)
        {
            centerOfMass += finger.root.position - body.position;
        }

        //Take the average of the fingers positions
        centerOfMass /= fingers.Length;

        body.centerOfMass = centerOfMass;

        //Set joint space for all the fingers
        for(int i = 0; i < fingers.Length; i++)
        {
            Transform selected = fingers[i].rootActive;
            List<Quaternion> cacheActives = new List<Quaternion>();
            while(selected != null)
            {
                cacheActives.Add(selected.localRotation);

                if (selected.childCount == 0)
                    break;

                selected = selected.GetChild(0);
            }

            fingers[i].cachedJointSpace = cacheActives.ToArray();
        }
    }

    private void Update()
    {
        //Get movement inputs
        inputVector = new Vector3(Input.GetAxisRaw("Horizontal"), 0f, Input.GetAxisRaw("Vertical"));

        PassiveMove();

        PassiveAim();

        Hover();
    }

    private void FixedUpdate()
    {
        //Aim();
        SyncAim();

        //Hover();

        //Movement();
        SyncMovement();

        SyncJointsWithIK();
    }

    [System.Obsolete("This method moves active body with force and syncs passive body with active body, therefore it works clumsy and wrong. Use PassiveAim for passive body and SyncAim for active body.")]
    /// <summary>
    /// Turns the active body and sync it with the passive body
    /// </summary>
    private void Aim()
    {
        if (!OnGround())
            return;

        //Calculate and apply standing torque
        //Palm.up must equal to palmUp vector
        Vector3 localUp = AverageTargetNormal().normalized;
        Vector3 aimUp = localUp.ClampAngle(cam.up, aimXRange.x, aimXRange.y);
        //Vector3 palmUp = Vector3.zero;
        //Debug.DrawRay(transform.position, localUp, Color.yellow);

        //Forward facing direction of the palm
        //Limit with angle degree range
        Vector3 localForward = Vector3.Cross(transform.right, localUp).normalized;
        Vector3 lookForward = localForward.ClampAngle(cam.forward, localUp, aimXRange.x, aimXRange.y).normalized;
        //Vector3 lookForward = Vector3.ProjectOnPlane(cam.forward, aimUp).normalized;
        //Debug.DrawRay(transform.position, localForward * 3f, Color.black);
        //Debug.DrawRay(transform.position, lookForward * 4f, Color.blue);

        Quaternion finalRot = Quaternion.RotateTowards(bodyPassive.rotation, Quaternion.LookRotation(lookForward, aimUp), Time.fixedDeltaTime * 45f);
        //finalRot = Quaternion.Lerp(finalRot, body.rotation, 0.2f);
        //bodyPassive.rotation = finalRot;

        Vector3 palmUp = Vector3.Cross(transform.up, aimUp) * torqueUpForceMultiplier;
        Vector3 palmForward = Vector3.Cross(transform.forward, lookForward) * torqueForwardForceMultiplier;
        //Vector3 palmForward = Vector3.zero;
        Vector3 palmTorque = pidTorque.UpdatePID(Time.fixedDeltaTime, body.angularVelocity, palmUp + palmForward, pidStoreTorque);
        //Debug.DrawRay(transform.position, palmTorque, Color.black);

        body.AddTorque(palmTorque);

        //bodyPassive.AddTorque(palmTorque);
        bodyPassive.rotation = body.rotation;
    }

    /// <summary>
    /// Syncs the active body rotation with the passive body rotation
    /// </summary>
    private void SyncAim()
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
    /// Turns the passive body to where the camera looks
    /// </summary>
    private void PassiveAim()
    {
        if (!OnGround()) //We are on falling or jumped
        {
            //Aim with keyboard
            return;
        }
        else //We are on ground
        {
            //Calculate surface normal according to finger placements and use it as local up vector
            Vector3 localUp = AverageTargetNormalPythagor().normalized;

            //Calculate forward vector with cross product on right and local up vector
            Vector3 localForward = Vector3.Cross(transform.right, localUp).normalized;
            //Clamp the vector from local forward to cam.forward with angle around local up vector
            Vector3 lookForward = VectorExtentions.ClampAngleAxis(localForward, cam.forward, localUp, aimXRange.x, aimXRange.y);

            //Use RotateTowards to smoothly rotate passive body
            Quaternion finalRot = Quaternion.RotateTowards(bodyPassive.rotation, Quaternion.LookRotation(lookForward, localUp), Time.deltaTime * 180f);
            bodyPassive.rotation = finalRot;
        }
    }

    /// <summary>
    /// Hovers the passive body
    /// </summary>
    private void Hover()
    {
        //Check if we are on ground
        if (!OnGround())
            return;

        //Calculate default hover height
        float goalHover = AverageTargetPosition().y + HOVER_HEIGHT;

        //Check if default hover height is reachable
        goalHover = Mathf.Min(goalHover, MaxHeight2(bodyPassive.position, bodyPassive.up));

        //COMMENTED DUE TO CONSTANT RISE SPEED, SLOW ADJUSTING ON STEEP SURFACES
        //float hoverForce = pidHover.UpdatePID(Time.deltaTime, bodyPassive.position.y, goalHover, pidStoreHover);
        //bodyPassive.position +=  Vector3.up * hoverForce * Time.deltaTime;

        //Hover passive body to the goal Y position. Lerp will help passive body to adjust as fast as the difference getting larger.
        Vector3 hoverPos = new Vector3(bodyPassive.position.x,
            Mathf.Lerp(bodyPassive.position.y, goalHover, Time.deltaTime * 20f),
            bodyPassive.position.z);
        bodyPassive.position = hoverPos;
    }

    [System.Obsolete("This method moves active body and syncs passive body with active body, therefore it works wrong on some situations. Use PassiveMove for passive body and SyncMovement for active body.")]
    /// <summary>
    /// Moves the active body with force and syncs X-Z coordinate with the passive body
    /// </summary>
    private void Movement()
    {
        if (!CanMove() || !OnGround())
        {
            bodyPassive.position = transform.position;
            return;
        }

        //Calculate and apply move force
        Vector3 velocity2D = transform.InverseTransformDirection(body.velocity);
        velocity2D.y = 0f;

        Vector3 flatCamForward = cam.forward;
        flatCamForward.y = 0f;
        float angle = Vector3.SignedAngle(Vector3.forward, flatCamForward, Vector3.up);
        Vector3 movement2D = Quaternion.AngleAxis(angle, Vector3.up) * inputVector;
        movement2D = Quaternion.AngleAxis(transform.localEulerAngles.x, transform.right) * movement2D;
        //Debug.DrawRay(transform.position, movement2D * 3f, Color.black);

        //bodyPassive.position += movement2D * Time.fixedDeltaTime * 2f;
        //bodyPassive.position = Vector3.Lerp(bodyPassive.position, body.position, 0.2f);

        Vector3 moveForce = pidMove.UpdatePID(Time.fixedDeltaTime, velocity2D, movement2D * moveForceMultiplier, pidStoreMove);
        body.AddForce(moveForce);

        Vector3 bodyPos = body.position;
        bodyPos.y = bodyPassive.position.y;
        bodyPassive.position = bodyPos;

        //bodyPassive.AddForce(moveForce);
    }

    /// <summary>
    /// Syncs the active body position with the passive body position
    /// </summary>
    private void SyncMovement()
    {
        //Check if we can move and are on ground
        if (!CanMove() || !OnGround())
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
        if (!CanMove() || !OnGround())
        {
            //Slowly syncs passive body position to active body position
            //Setting it as body.position cause passive body to teleport
            bodyPassive.position = Vector3.Lerp(bodyPassive.position, body.position, Time.deltaTime * 10f);
            return;
        }

        //Rotate input vector to where the camera looks
        Vector3 flatCamForward = cam.forward;
        flatCamForward.y = 0f;
        float angle = Vector3.SignedAngle(Vector3.forward, flatCamForward, Vector3.up);
        Vector3 movement2D = Quaternion.AngleAxis(angle, Vector3.up) * inputVector;
        movement2D = Quaternion.AngleAxis(transform.localEulerAngles.x, transform.right) * movement2D;

        //Set passive pos to active body position
        //Can be used either of passivePos calculations
        //passivePos = Vector3.Lerp(passivePos, body.position, (body.position - passivePos).magnitude / MAX_POS_DIFF);
        passivePos = body.position;

        //Calculate final pos with constant distance to prevent passive body to go far away from active body
        Vector3 finalPos = passivePos + movement2D * MAX_POS_DIFF;

        //Move with lerp to move passive body faster as distance between passive body and final position gets bigger
        bodyPassive.position = Vector3.Lerp(bodyPassive.position, finalPos, Time.deltaTime * 3f);
    }

    /// <summary>
    /// Syncs joints rotation with ik rotation
    /// </summary>
    private void SyncJointsWithIK()
    {
        foreach(IKHolder holder in fingers)
        {
            //Can be done with recursive method
            //Set first variables
            int i = 0;
            Quaternion cacheRot = holder.cachedJointSpace[i];
            Transform currentParent = holder.root;
            Transform currentActiveParent = holder.rootActive;
            while(currentParent != null && currentActiveParent != null)
            {
                //Check if active bone has joint (finger tips doesnt have joint)
                if (currentActiveParent.TryGetComponent(out ConfigurableJoint joint))
                {
                    //Set target rotation of the joint
                    joint.SetTargetRotationLocal(currentParent.localRotation, cacheRot);
                }
                else //We reached the end, no need to continue
                    break;

                //There is no more children left
                if (currentParent.childCount == 0 || currentActiveParent.childCount == 0)
                    break;

                //Set next patch
                currentParent = currentParent.GetChild(0);
                currentActiveParent = currentActiveParent.GetChild(0);
                cacheRot = holder.cachedJointSpace[++i];
            }
        }
    }

    /// <summary>
    /// Average position of all IK targets.
    /// </summary>
    /// <returns>Average position (world coordinate)</returns>
    private Vector3 AverageTargetPosition()
    {
        //If there is no fingers, return hand pivot position
        if (fingers.Length == 0)
            return transform.position;

        //Calculate and return fingers average target position
        Vector3 total = Vector3.zero;
        foreach(IKHolder ik in fingers)
        {
            total += ik.solver.worldPlacePoint;
        }
        return total / fingers.Length;
    }

    [System.Obsolete("This method calculates wrong values on some cases. Use AverageTargetNormalPythagor insted.")]
    /// <summary>
    /// Average normal vector of all IK targets (not normalized).
    /// </summary>
    /// <returns>Average normal vector (not normalized)</returns>
    private Vector3 AverageTargetNormal()
    {
        //Calculation done with repetitive triangle surface normal calculation
        //Lets say we have ABC triangle, A is our average position and B is our helper object
        //We will iterate C point for every IK solver and adding to the normal vector

        Vector3 helperPoint = transform.position - transform.forward * 1.5f + transform.up + transform.right;
        Vector3 avg = transform.position - transform.forward * 1.5f + transform.up - transform.right;
        //Vector3 avg = AverageTargetPosition();

        Vector3 normal = Vector3.zero;
        foreach (IKHolder ik in fingers)
        {
            if (!ik.solver.onGround)
                continue;

            Vector3 ikNormal = Vector3.Cross((ik.solver.worldPlacePoint - avg).normalized, (ik.solver.worldPlacePoint - helperPoint).normalized);
            //Vector3 ikNormal = bodyPassive.position - ik.solver.worldPlacePoint;
            //Vector3 cross = Vector3.Cross(ik.solver.hitNormal, ikNormal);
            //ikNormal = Vector3.Cross(ikNormal, cross).normalized;
            if (Vector3.Dot(ikNormal, ik.solver.rayDir) > 0f)
                ikNormal *= -1f;

            Debug.DrawRay(ik.solver.worldPlacePoint, ikNormal, Color.blue);
            //Debug.DrawLine(ik.solver.worldPlacePoint, avg, Color.red);
            //Debug.DrawLine(ik.solver.worldPlacePoint, helperPoint, Color.red);
            Debug.DrawLine(ik.solver.worldPlacePoint, bodyPassive.position, Color.red);

            normal += ikNormal;
        }

        return normal;
    }

    /// <summary>
    /// Average normal vector of all IK targets (not normalized).
    /// </summary>
    /// <returns>Average normal vector (not normalized)</returns>
    private Vector3 AverageTargetNormalPythagor()
    {
        //Calculation done with pythagor theorem
        //Lets assume we have ABC triangle where B is right angle
        //|AB| = a, |AC| = c, |BC| = b

        Vector3 normal = Vector3.zero;
        foreach (IKHolder ik in fingers)
        {
            //Calculate 'c' side
            Vector3 ikNormal = (bodyPassive.position - ik.root.position) * ik.extentMax;

            //Angle of 'C'. We will take 'a' side as maximum extent of the finger
            float deg = Mathf.Asin(ik.extentMax / ikNormal.magnitude) * Mathf.Rad2Deg;

            //Surface normal of the rayHit (also 'a' side)
            Vector3 surfaceNormal = ik.solver.hitNormal;
            //Rotate surface normal with angle of 'C' on cross of 'a' side and 'c' side
            ikNormal = Quaternion.AngleAxis(deg, Vector3.Cross(surfaceNormal, ikNormal)) * surfaceNormal.normalized;

            //Debug lines to see fingers normal direction
            //Debug.DrawRay(ik.solver.worldPlacePoint, ikNormal, Color.blue);
            //Debug.DrawRay(ik.root.position, Vector3.Cross(surfaceNormal, ikNormal), Color.yellow);
            //Debug.DrawRay(ik.root.position, (bodyPassive.position - ik.root.position) * ik.extentMax, Color.red);

            //Finally, add last normal
            normal += ikNormal;
        }

        return normal;
    }

    [System.Obsolete("Calculates maximum height with patches of 2 fingers. Won't work stable with only one finger. Use MaxHeight2 instead.")]
    /// <summary>
    /// (OBSOLOTE) Calculates maximum height as if fingers are on ground
    /// </summary>
    /// <returns></returns>
    private float MaxHeight1()
    {
        float maxReach = float.MaxValue;
        for(int i = 0; i < fingers.Length; i++)
        {
            //Take two fingers
            IKHolder f1 = fingers[i];
            IKHolder f2 = fingers[(i + 1) % fingers.Length];

            //Difference between root positions
            Vector3 diffRoot = f2.root.position - f1.root.position;

            //Set dominant and submissive fingers (can be used just f1 and f2, nothing special)
            IKHolder domHolder = f1, subHolder = f2;

            //Calculate where difference between roots end up on submissive holder
            Vector3 lastPoint = f1.solver.worldPlacePoint + diffRoot;

            //Calculate current dominant holder distance
            float currentDomDistance = (domHolder.root.position - domHolder.solver.worldPlacePoint).magnitude;

            //First calculate submissive max distance (Law of sines)
            float sideC = Vector3.Distance(lastPoint, subHolder.solver.worldPlacePoint);
            float maxExtentUp = 0;
            if(sideC <= subHolder.extentMax) //last point is inside of submissive holder reach
            {
                //Calculate vector from submissive to last point
                Vector3 subToLast = lastPoint - subHolder.solver.worldPlacePoint;

                float angleB = Vector3.Angle(-subToLast, transform.up);
                if(Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
                {
                    //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                    //Just take extent
                    if (angleB > 90f)
                        maxExtentUp = subHolder.extentMax - sideC;
                    //If B equals to 0 degree then up direction and subToLast vectors are in opposite direction
                    //Reverse the extent
                    else
                        maxExtentUp = -sideC;
                }
                else
                {
                    //Since we know two sides and one angle of triangle, we can calculate rest of it with law of sine
                    //One side is 'sideC' other side is maximum extent of finger
                    //With this calculation we can calculate how far away we can move on transform.up direction
                    float angleC = Mathf.Asin((sideC * Mathf.Sin(angleB * Mathf.Deg2Rad)) / subHolder.extentMax) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (subHolder.extentMax * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            //Debug.DrawRay(subHolder.solver.worldPlacePoint, subToLast, Color.yellow);

            //Finally, see if which one is smaller to not exceed other finger maximum extent
            float maxReachable = Mathf.Min(maxExtentUp, domHolder.extentMax - currentDomDistance, subHolder.extentMax);
            //Debug.DrawRay(subHolder.solver.worldPlacePoint, transform.up * maxReachable, Color.green);
            maxReach = Mathf.Min(maxReach, maxReachable);
        }

        return (transform.position + transform.up * maxReach).y;
    }

    /// <summary>
    /// Calculates maximum height as if fingers are on ground
    /// </summary>
    /// <param name="upDir">Direction to calculate maximum extend</param>
    /// <returns></returns>
    private float MaxHeight2(Vector3 pos, Vector3 upDir)
    {
        float maxReach = float.MaxValue;
        for (int i = 0; i < fingers.Length; i++)
        {
            IKHolder f1 = fingers[i];
            //Difference between ik target and bone root
            Vector3 diffRoot = f1.root.position - f1.solver.worldPlacePoint;

            //Distance between root bone and ik target (used as side 'c')
            float currentDist = diffRoot.magnitude;

            //First calculate finger max distance (Law of sines)
            float maxExtentUp = 0;
            if (currentDist <= f1.extentMax)
            {
                float angleB = Vector3.Angle(-diffRoot, upDir);
                if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
                {
                    //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                    //Just take extent
                    if (angleB > 90f)
                        maxExtentUp = f1.extentMax - currentDist;
                    //If B equals to 0 degree then up direction and subToLast vectors are in opposite direction
                    //Reverse the extent
                    else
                        maxExtentUp = -currentDist;
                }
                else
                {
                    //Since we know two sides and one angle of triangle, we can calculate rest of it with law of sine
                    //One side is 'currentDist' other side is maximum extent of finger
                    //With this calculation we can calculate how far away we can move on 'upDir' direction
                    float angleC = Mathf.Asin((currentDist * Mathf.Sin(angleB * Mathf.Deg2Rad)) / f1.extentMax) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (f1.extentMax * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            //Debug.DrawRay(f1.solver.worldPlacePoint, diffRoot, Color.yellow);

            maxReach = Mathf.Min(maxReach, maxExtentUp, f1.extentMax - currentDist);
        }

        return (pos + upDir * maxReach).y;
    }

    /// <summary>
    /// Returns true if any finger is on ground
    /// </summary>
    /// <returns></returns>
    private bool OnGround()
    {
        //If at least one finger is on ground, return true
        bool onGround = false;
        foreach (IKHolder holder in fingers)
        {
            if (holder.solver.onGround)
            {
                onGround = true;
                break;
            }
        }

        return onGround;
    }

    /// <summary>
    /// Returns true if any finger is stationary
    /// </summary>
    /// <returns></returns>
    private bool CanMove()
    {
        //If at least one finger is stationary, return true
        bool canMove = false;
        foreach (IKHolder holder in fingers)
        {
            if (!holder.solver.isMoving)
            {
                canMove = true;
                break;
            }
        }

        return canMove;
    }
}
