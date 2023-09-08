using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations.Rigging;
using Extension;

namespace BIK.FullActive
{
    public class FullIKInfo : MonoBehaviour
    {
        //public Transform root; //IK passive body root
        //public Transform rootActive; //IK active body root (joint root)
        [SerializeField, Header("Editor Only")] private Transform passiveRoot;
        [SerializeField] private Transform activeRoot;

        [Header("Root to tip")] public Transform[] passiveSegments; //IK passive body segments
        [Header("Root to tip")] public Transform[] activeSegments; //IK active body segments
        public FullTargetIKSolver solver; //IK solver
        public float extentMax; //Maximum extent of the finger
        public float extentMin; //Minimum extent of the finger

        [HideInInspector]
        public Quaternion[] cachedJointSpace; //Cached joint spaces (used to rotates the joints)

        private int lastActiveSegment; //Last gameObject.ActiveSelf object id on segments array

        //public bool IsIKAvailable => activeSegments[0].gameObject.activeSelf;
        public bool IsIKAvailable => lastActiveSegment >= 0;
        public Transform GetPassiveRoot => passiveSegments[0];
        public Transform GetActiveRoot => activeSegments[0];
        //public Transform GetPassiveTip => passiveSegments[lastActiveSegment]; //Not in use
        public Transform GetActiveTip => activeSegments[Mathf.Min(lastActiveSegment + 1, activeSegments.Length - 1)];

#if UNITY_EDITOR
        //Easy way to calculate max extent
        [ContextMenu("IK length")]
        private void CalculateLength()
        {
            if(passiveSegments.Length > 0)
                Debug.Log(passiveSegments[0].name + " length: " + SegmentsLength(passiveSegments));
        }

        [ContextMenu("Automatic segment fill")]
        private void AutoFillSegments()
        {
            if(passiveRoot != null)
            {
                List<Transform> s = new List<Transform>();
                Transform t = passiveRoot;
                while(t != null)
                {
                    s.Add(t);

                    if (t.childCount > 0)
                        t = t.GetChild(0);
                    else
                        break;
                }
                passiveSegments = s.ToArray();
            }

            if(activeRoot != null)
            {
                List<Transform> s = new List<Transform>();
                Transform t = activeRoot;
                while (t != null)
                {
                    s.Add(t);

                    if (t.childCount > 0)
                        t = t.GetChild(0);
                    else
                        break;
                }
                activeSegments = s.ToArray();
            }
        }
#endif

        public void Initialize()
        {
#if UNITY_EDITOR
            if(activeSegments.Length != passiveSegments.Length)
            {
                Debug.LogWarning("Active and passive segments length is not equal on " + transform.name + ". This can cause misbehaviour", transform);
            }
#endif
            lastActiveSegment = -1;

            cachedJointSpace = new Quaternion[activeSegments.Length];
            for(int i = 0; i < cachedJointSpace.Length; i++)
            {
                cachedJointSpace[i] = activeSegments[i].localRotation;

                if (activeSegments[i].gameObject.activeSelf)
                    lastActiveSegment = i;
            }
        }

        public void FireSegment()
        {
            if (!IsIKAvailable)
                return;

            Transform selectedSegment = activeSegments[lastActiveSegment];
            Transform selectedPassive = passiveSegments[lastActiveSegment];

            GameObject clone = Instantiate(selectedSegment.gameObject, selectedSegment.position, selectedSegment.rotation);
            Destroy(clone.GetComponent<Joint>()); //Remove joint from the clone
            Rigidbody body = clone.GetComponent<Rigidbody>();
            body.AddForce(selectedSegment.forward * 50f, ForceMode.Impulse);
            body.useGravity = true;

            if(selectedSegment.parent.TryGetComponent(out CollisionDetector cd))
                cd.enabled = true;

            selectedSegment.gameObject.SetActive(false);
            selectedSegment.GetComponent<CollisionDetector>().enabled = false;
            selectedPassive.gameObject.SetActive(false);
            solver.ChangeRayLength(GetLength(false));
            solver.ChangeTipCollider(selectedSegment.parent.GetComponent<Collider>());

            ChainIKConstraint chainIK = solver.GetComponent<ChainIKConstraint>();
            if(selectedPassive.parent != chainIK.data.root)
            {
                chainIK.data.tip = selectedPassive;
            }
            else if(solver.TryGetComponent(out MultiAimConstraint multiAim))
            {
                multiAim.weight = 1f;
                chainIK.weight = 0f;
            }

            lastActiveSegment--;
        }

        public void SyncJoints()
        {
            for(int i = 0; i < activeSegments.Length; i++)
            {
                if (activeSegments[i].TryGetComponent(out ConfigurableJoint joint))
                {
                    joint.SetTargetRotationLocal(passiveSegments[i].localRotation, cachedJointSpace[i]);
                }
            }
        }

        public float CalculateReachableRange(Vector3 upDir)
        {
            //Difference between ik target and bone root
            Vector3 diffRoot = GetPassiveRoot.position - solver.worldPlacePoint;

            //Distance between root bone and ik target (used as side 'c')
            float currentDist = diffRoot.magnitude;

            //First calculate finger max distance (Law of sines)
            float maxExtentUp = 0;
            if (currentDist <= solver.RayLength)
            {
                float angleB = Vector3.Angle(-diffRoot, upDir);
                if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
                {
                    //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                    //Just take extent
                    if (angleB > 90f)
                        maxExtentUp = solver.RayLength - currentDist;
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
                    float angleC = Mathf.Asin((currentDist * Mathf.Sin(angleB * Mathf.Deg2Rad)) / solver.RayLength) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (solver.RayLength * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            return Mathf.Min(maxExtentUp, solver.RayLength - currentDist);
        }

        public Vector3 CalculateNormal(Vector3 cSide)
        {
            //Calculation done with pythagor theorem
            //Lets assume we have ABC triangle where B is right angle
            //|AB| = a, |AC| = c, |BC| = b

            //Angle of 'C'. We will take 'a' side as maximum extent of the finger
            float deg = Mathf.Asin(solver.RayLength / cSide.magnitude) * Mathf.Rad2Deg;

            //Surface normal of the rayHit (also 'a' side)
            Vector3 surfaceNormal = solver.hitNormal;
            //Rotate surface normal with angle of 'C' on cross of 'a' side and 'c' side
            cSide = Quaternion.AngleAxis(deg, Vector3.Cross(surfaceNormal, cSide)) * surfaceNormal;

            //Debug lines to see fingers normal direction
            //Debug.DrawRay(ik.solver.worldPlacePoint, ikNormal, Color.blue);
            //Debug.DrawRay(ik.root.position, Vector3.Cross(surfaceNormal, ikNormal), Color.yellow);
            //Debug.DrawRay(ik.root.position, (bodyPassive.position - ik.root.position) * ik.extentMax, Color.red);

            return cSide;
        }

        public Vector3 TipProjectedOnRoot()
        {
            Vector3 local = GetActiveRoot.InverseTransformPoint(GetActiveTip.position);
            local.y = 0;

            return GetActiveRoot.TransformPoint(local);
        }

        public void SetConnectedMassScale(float v)
        {
            for(int i = 0; i < activeSegments.Length; i++)
            {
                if (activeSegments[i].TryGetComponent(out Joint j))
                {
                    j.connectedMassScale = v;
                }
            }
        }

        public void SetSlerpForce(float f)
        {
            for (int i = 0; i < activeSegments.Length; i++)
            {
                if (activeSegments[i].TryGetComponent(out ConfigurableJoint cj))
                {
                    JointDrive slerpD = cj.slerpDrive;
                    slerpD.positionSpring = f;
                    cj.slerpDrive = slerpD;
                }
            }
        }

        public float GetLength(bool calculateInactive = true)
        {
            return SegmentsLength(activeSegments, calculateInactive);
        }

        //Calculates total distance between last children position and root position
        private float SegmentsLength(Transform[] segments, bool calculateInactive = true)
        {
            float result = 0f;
            for(int i = 0; i < segments.Length - 1; i++)
            {
                if (!calculateInactive && !segments[i].gameObject.activeSelf)
                    break;

                result = Vector3.Distance(segments[i].position, segments[i + 1].position);
            }

            return result;
        }
    }
}
