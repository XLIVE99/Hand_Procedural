using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations.Rigging;
using Extension;

namespace HandWar
{
    public class IKInfo : MonoBehaviour
    {
        public Transform root; //IK passive body root
        public Transform rootActive; //IK active body root (joint root)
        public TargetIKSolver solver; //IK solver
        public float extentMax; //Maximum extent of the finger
        //public float extentMin; //Minimum extent of the finger (not used)

        [HideInInspector]
        public Quaternion[] cachedJointSpace; //Cached joint spaces (used to rotates the joints)

#if UNITY_EDITOR
        //Easy way to calculate max extent
        [ContextMenu("IK length")]
        private void CalculateLength()
        {
            Debug.Log(root.name + " length: " + GetLength());
        }
#endif

        public bool IsIKAvailable => rootActive.gameObject.activeSelf;

        public void Initialize()
        {
            Transform selected = rootActive;
            List<Quaternion> cacheActives = new List<Quaternion>();
            while (selected != null)
            {
                cacheActives.Add(selected.localRotation);

                if (selected.childCount == 0)
                    break;

                selected = selected.GetChild(0);
            }

            cachedJointSpace = cacheActives.ToArray();
        }

        public void FireSegment()
        {
            Transform selectedSegment = rootActive;
            Transform selectedPassive = root;
            while (selectedSegment.childCount > 0 && selectedSegment.GetChild(0).gameObject.activeSelf)
            {
                selectedSegment = selectedSegment.GetChild(0);
                selectedPassive = selectedPassive.GetChild(0);
            }

            GameObject clone = Instantiate(selectedSegment.gameObject, selectedSegment.position, selectedSegment.rotation);
            Destroy(clone.GetComponent<Joint>()); //Remove joint
            Rigidbody body = clone.GetComponent<Rigidbody>();
            body.AddForce(selectedSegment.up * 5f, ForceMode.Impulse);
            body.useGravity = true;

            selectedSegment.gameObject.SetActive(false);
            solver.ChangeRayLength(GetLength(false));

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
        }

        public void SyncJoints()
        {
            //Can be done with recursive method
            //Set first variables
            int i = 0;
            Quaternion cacheRot = cachedJointSpace[i];
            Transform currentParent = root;
            Transform currentActiveParent = rootActive;
            while (currentParent != null && currentActiveParent != null)
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
                cacheRot = cachedJointSpace[++i];
            }
        }

        public float CalculateReachableRange(Vector3 upDir)
        {
            //Difference between ik target and bone root
            Vector3 diffRoot = root.position - solver.worldPlacePoint;

            //Distance between root bone and ik target (used as side 'c')
            float currentDist = diffRoot.magnitude;

            //First calculate finger max distance (Law of sines)
            float maxExtentUp = 0;
            if (currentDist <= extentMax)
            {
                float angleB = Vector3.Angle(-diffRoot, upDir);
                if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
                {
                    //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                    //Just take extent
                    if (angleB > 90f)
                        maxExtentUp = extentMax - currentDist;
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
                    float angleC = Mathf.Asin((currentDist * Mathf.Sin(angleB * Mathf.Deg2Rad)) / extentMax) * Mathf.Rad2Deg;
                    float angleA = 180f - angleB - angleC;
                    maxExtentUp = (extentMax * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                }
            }

            return Mathf.Min(maxExtentUp, extentMax - currentDist);
        }

        public Vector3 CalculateNormal(Vector3 cSide)
        {
            //Angle of 'C'. We will take 'a' side as maximum extent of the finger
            float deg = Mathf.Asin(extentMax / cSide.magnitude) * Mathf.Rad2Deg;

            //Surface normal of the rayHit (also 'a' side)
            Vector3 surfaceNormal = solver.hitNormal;
            //Rotate surface normal with angle of 'C' on cross of 'a' side and 'c' side
            cSide = Quaternion.AngleAxis(deg, Vector3.Cross(surfaceNormal, cSide)) * surfaceNormal;

            //Debug lines to see fingers normal direction
            //Debug.DrawRay(ik.solver.worldPlacePoint, ikNormal, Color.blue);
            //Debug.DrawRay(ik.root.position, Vector3.Cross(surfaceNormal, ikNormal), Color.yellow);
            //Debug.DrawRay(ik.root.position, (bodyPassive.position - ik.root.position) * ik.extentMax, Color.red);

            //Finally, add last normal
            return cSide;
        }

        public float GetLength(bool calculateInactive = true)
        {
            return recursiveLength(rootActive, calculateInactive);
        }

        //Calculates total distance between last children position and root position
        private float recursiveLength(Transform parent, bool calculateInactive = true)
        {
            if (!calculateInactive && !parent.gameObject.activeSelf)
                return 0f;

            if (parent.childCount <= 0)
                return 0f;
            else
            {
                return Vector3.Distance(parent.position, parent.GetChild(0).position) + recursiveLength(parent.GetChild(0), calculateInactive);
            }
        }
    }
}
