using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PlayerController : MonoBehaviour
{
    [SerializeField] private float moveForceMultiplier = 10f;
    [SerializeField] private float hoverForceMultiplier = 10f;
    [SerializeField] private float torqueForwardForceMultiplier = 10f, torqueUpForceMultiplier = 4f;
    [SerializeField] private Vector2 aimXRange;
    [SerializeField] private ikHolder[] fingers;

    [SerializeField] private PIDController pidMove, pidTorque, pidHover;

    private Rigidbody body;
    private Transform cam;

    private PIDStore<Vector3> pidStoreMove;
    private PIDStore<Vector3> pidStoreTorque;
    private PIDStore<float> pidStoreHover;

    private Vector3 inputVector;

    private const float HOVER_HEIGHT = 2f;

    [System.Serializable]
    public struct ikHolder
    {
#if UNITY_EDITOR
        public string ikName;
#endif
        public Transform root;
        public TargetIKSolver solver;
        public float extentMax;
        public float extentMin;
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        const float DISTANCE = 2f;
        const float RESOLUTION = 2f;
        float steps = 1f / RESOLUTION;

        Gizmos.color = Color.green;
        for(float i = aimXRange.x; i < aimXRange.y; i += steps)
        {
            Gizmos.DrawLine(transform.position + Quaternion.AngleAxis(i, -transform.right) * transform.forward * DISTANCE,
                transform.position + Quaternion.AngleAxis(i + steps, -transform.right) * transform.forward * DISTANCE);
        }
        Gizmos.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(aimXRange.x, -transform.right) * transform.forward * DISTANCE);
        Gizmos.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(aimXRange.y, -transform.right) * transform.forward * DISTANCE);
    }

    [ContextMenu("Fingers length")]
    private void CalculateFingerLengths()
    {
        foreach(ikHolder finger in fingers)
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

        Vector3 centerOfMass = Vector3.zero;

        //Calculate mid point based on all finger root's positions
        foreach(ikHolder finger in fingers)
        {
            centerOfMass += finger.root.localPosition;
        }

        centerOfMass /= fingers.Length;

        body.centerOfMass = centerOfMass;
    }

    private void Update()
    {
        inputVector = new Vector3(Input.GetAxisRaw("Horizontal"), 0f, Input.GetAxisRaw("Vertical"));
    }

    private void FixedUpdate()
    {
        //Note: We don't need to use AddForceAtPosition because we will constantly update body.centerOfMass

        Aim();

        Hover();

        Movement();
    }

    private void Aim()
    {
        //Calculate and apply standing torque
        //Palm.up must equal to palmUp vector
        Vector3 localUp = AverageTargetNormal().normalized;
        Vector3 aimUp = localUp.ClampAngle(cam.up, aimXRange.x, aimXRange.y);
        Vector3 palmUp = Vector3.Cross(transform.up, aimUp) * torqueUpForceMultiplier;
        //Debug.DrawRay(transform.position, localUp, Color.yellow);

        //Forward facing direction of the palm
        //Limit with angle degree range
        Vector3 localForward = Vector3.Cross(transform.right, AverageTargetNormal()).normalized;
        Vector3 camForward = cam.forward;
        camForward.y = 0f;
        Vector3 lookForward = localForward.ClampAngle(cam.forward, aimXRange.x, aimXRange.y).normalized;
        //Debug.DrawRay(transform.position, localForward * 3f, Color.black);
        //Debug.DrawRay(transform.position, lookForward * 4f, Color.blue);

        Vector3 palmForward = Vector3.Cross(transform.forward, lookForward) * torqueForwardForceMultiplier;
        Vector3 palmTorque = pidTorque.UpdatePID(Time.fixedDeltaTime, body.angularVelocity, palmUp + palmForward, pidStoreTorque);
        //Debug.DrawRay(transform.position, palmTorque, Color.black);

        body.AddTorque(palmTorque);
    }

    private void Hover()
    {
        //Calculate and apply standing force to hover the palm
        //Calculate hover height
        float goalHover = AverageTargetPosition().y + HOVER_HEIGHT;

        //Limit goalHover according to ik extents
        //goalHover = LimitHeight(goalHover);
        goalHover = Mathf.Min(goalHover, MaxHeight2());
        Vector3 position2D = new Vector3(transform.position.x, 0, transform.position.z);
        //Debug.DrawLine(transform.position, position2D + Vector3.up * goalHover, Color.green);

        float hoverForce = pidHover.UpdatePID(Time.fixedDeltaTime, body.position.y, goalHover, pidStoreHover);
        body.AddForce(Vector3.up * hoverForce * hoverForceMultiplier);
    }

    private void Movement()
    {
        //Calculate and apply move force
        Vector3 velocity2D = body.velocity;
        velocity2D.y = 0f;

        Vector3 flatCamForward = cam.forward;
        flatCamForward.y = 0f;
        float angle = Vector3.SignedAngle(Vector3.forward, flatCamForward, Vector3.up);

        Vector3 moveForce = pidMove.UpdatePID(Time.fixedDeltaTime, velocity2D, Quaternion.AngleAxis(angle, Vector3.up) * inputVector * moveForceMultiplier, pidStoreMove);
        body.AddForce(moveForce);
    }

    /// <summary>
    /// Average position of all IK targets.
    /// </summary>
    /// <returns>Average position (world coordinate)</returns>
    private Vector3 AverageTargetPosition()
    {
        if (fingers.Length == 0)
            return transform.position;
        Vector3 total = Vector3.zero;
        foreach(ikHolder ik in fingers)
        {
            total += ik.solver.worldPlacePoint;
        }
        return total / fingers.Length;
    }

    /// <summary>
    /// Average normal vector of all IK targets (nor normalized).
    /// </summary>
    /// <returns>Average normal vector (not normalized)</returns>
    private Vector3 AverageTargetNormal()
    {
        //Calculation done with repetitive triangle surface normal calculation
        //Lets say we have ABC triangle, A is our average position and B is our helper object
        //We will iterate C point for every IK solver and adding to the normal vector

        Vector3 helperPoint = transform.position + Vector3.down * 1.5f;
        Vector3 avg = AverageTargetPosition();

        Vector3 normal = Vector3.zero;
        foreach (ikHolder ik in fingers)
        {
            Vector3 ikNormal = Vector3.Cross((ik.solver.worldPlacePoint - avg).normalized, (ik.solver.worldPlacePoint - helperPoint).normalized);
            if (Vector3.Dot(ikNormal, ik.solver.rayDir) > 0f)
                ikNormal *= -1f;

            //Debug.DrawRay(ik.solver.worldPlacePoint, ikNormal, Color.blue);
            //Debug.DrawLine(ik.solver.worldPlacePoint, avg, Color.red);
            //Debug.DrawLine(ik.solver.worldPlacePoint, helperPoint, Color.red);

            normal += ikNormal;
        }

        return normal;
    }

    //Requires at least 2 fingers (OBSOLOTE)
    private float MaxHeight1()
    {
        float maxReach = float.MaxValue;
        for(int i = 0; i < fingers.Length; i++)
        {
            ikHolder f1 = fingers[i];
            ikHolder f2 = fingers[(i + 1) % fingers.Length];
            Vector3 diffRoot = f2.root.position - f1.root.position;
            //Debug.DrawRay(f1.root.position, diffRoot, Color.black);
            ikHolder domHolder = f1, subHolder = f2;
            Vector3 lastPoint = f1.solver.worldPlacePoint + diffRoot;
            //Debug.DrawLine(f1.solver.worldPlacePoint, lastPoint, Color.black);

            float currentDomDistance = (domHolder.root.position - domHolder.solver.worldPlacePoint).magnitude;

            //First calculate dominant max distance (Law of sines)
            Vector3 subToLast = lastPoint - subHolder.solver.worldPlacePoint;
            float sideC = Vector3.Distance(lastPoint, subHolder.solver.worldPlacePoint);
            float maxExtentUp = 0;
            if(sideC <= subHolder.extentMax)
            {
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
                    float angleC = Mathf.Asin((sideC * Mathf.Sin(angleB * Mathf.Deg2Rad)) / subHolder.extentMax) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (subHolder.extentMax * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            //Debug.DrawRay(subHolder.solver.worldPlacePoint, subToLast, Color.yellow);

            float maxReachable = Mathf.Min(maxExtentUp, domHolder.extentMax - currentDomDistance, subHolder.extentMax);
            //Debug.DrawRay(subHolder.solver.worldPlacePoint, transform.up * maxReachable, Color.green);
            maxReach = Mathf.Min(maxReach, maxReachable);
        }

        return (transform.position + transform.up * maxReach).y;
    }

    //Can work with only a finger but requires global up direction
    private float MaxHeight2()
    {
        float maxReach = float.MaxValue;
        for (int i = 0; i < fingers.Length; i++)
        {
            ikHolder f1 = fingers[i];
            //Vector3 avgNormal = AverageTargetNormal().normalized;
            Vector3 diffRoot = f1.root.position - f1.solver.worldPlacePoint;

            //Debug.DrawRay(f1.root.position + diffRoot * 0.5f, transform.up, Color.green);
            float currentDist = (f1.root.position - f1.solver.worldPlacePoint).magnitude;

            //First calculate dominant max distance (Law of sines)
            float sideC = diffRoot.magnitude;
            float maxExtentUp = 0;
            if (sideC <= f1.extentMax)
            {
                float angleB = Vector3.Angle(-diffRoot, transform.up);
                if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
                {
                    //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                    //Just take extent
                    if (angleB > 90f)
                        maxExtentUp = f1.extentMax - sideC;
                    //If B equals to 0 degree then up direction and subToLast vectors are in opposite direction
                    //Reverse the extent
                    else
                        maxExtentUp = -sideC;
                }
                else
                {
                    float angleC = Mathf.Asin((sideC * Mathf.Sin(angleB * Mathf.Deg2Rad)) / f1.extentMax) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (f1.extentMax * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            //Debug.DrawRay(f1.solver.worldPlacePoint, diffRoot, Color.yellow);

            float maxReachable = Mathf.Min(maxExtentUp, f1.extentMax - currentDist);
            //Debug.DrawRay(f1.solver.worldPlacePoint, transform.up * maxReachable, Color.gray);

            maxReach = Mathf.Min(maxReach, maxReachable);
        }

        return (transform.position + transform.up * maxReach).y;
    }

    private float LimitHeight(float h)
    {
        float error = 0f;
        foreach(ikHolder holder in fingers)
        {
            float diffMagnitude = (holder.root.position - holder.solver.worldPlacePoint).magnitude;
            if (diffMagnitude > holder.extentMax && error < diffMagnitude - holder.extentMax)
            {
                error = diffMagnitude - holder.extentMax;
            }
        }

        return h - error;
    }

    /// <summary>
    /// Returns average maximum height point of iks
    /// </summary>
    /// <returns>Maximum height point (world coordinate)</returns>
    private Vector3 AverageMaxHeight()
    {
        if (fingers.Length == 0)
            return transform.position;

        Vector3 avgNormal = AverageTargetNormal();
        Vector3 maxHeight = Vector3.zero;
        foreach(ikHolder ik in fingers)
        {
            maxHeight += avgNormal.normalized * ik.extentMax;
        }

        return maxHeight / fingers.Length;
    }

    /// <summary>
    /// Returns average minimum height point of iks
    /// </summary>
    /// <returns>Minimum height point (world coordinate)</returns>
    private Vector3 AverageMinHeight()
    {
        if (fingers.Length == 0)
            return transform.position;

        Vector3 minHeight = Vector3.zero;
        foreach(ikHolder ik in fingers)
        {
            minHeight += (ik.root.position - ik.solver.worldPlacePoint).normalized * ik.extentMin;
        }

        return minHeight / fingers.Length;
    }
}
