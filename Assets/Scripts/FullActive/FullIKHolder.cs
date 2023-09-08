using UnityEngine;
using UnityEngine.Animations.Rigging;

namespace BIK.FullActive
{
    public class FullIKHolder : MonoBehaviour
    {
        [SerializeField] private FullIKInfo[] infos;
        [SerializeField] private RigBuilder rigBuilder;

        public int TotalIK => infos.Length;

        public void Start()
        {
            foreach (FullIKInfo info in infos)
                info.Initialize();
        }

        public void FireSegment(FullIKInfo finger)
        {
            finger.FireSegment();

            //In order to build properly, animator must have default animation
            //Bones must be in default rotation (or position if you modifying bones position with IK) in that default animation
            rigBuilder.Build();
        }

        /// <summary>
        /// Returns active finger count
        /// </summary>
        /// <returns></returns>
        public int GetAvailableIKsCount()
        {
            int i = 0;
            foreach(FullIKInfo info in infos)
            {
                if (info.IsIKAvailable)
                    i++;
            }
            return i;
        }

        /// <summary>
        /// Returns grounded finger count
        /// </summary>
        /// <returns></returns>
        public int GetGroundedIKCount()
        {
            int i = 0;
            foreach (FullIKInfo info in infos)
            {
                if (info.IsIKAvailable && info.solver.onGround && !info.solver.rayOverriden) //Aiming finger means not on ground
                    i++;
            }
            return i;
        }

        /// <summary>
        /// Returns a ready to fire finger
        /// </summary>
        /// <returns></returns>
        public FullIKInfo GetAvailableIK()
        {
            foreach(FullIKInfo info in infos)
            {
                if(info.IsIKAvailable)
                {
                    return info;
                }
            }

            return null;
        }

        public void AddGlobalRotation(float angle, Vector3 axis)
        {
            foreach(FullIKInfo info in infos)
            {
                if (info.IsIKAvailable)
                    info.solver.RotateSecondRayGlobal(angle, axis);
            }
        }

        public void AddLocalRotation(float angle, Vector3 axis)
        {
            foreach (FullIKInfo info in infos)
            {
                if (info.IsIKAvailable)
                    info.solver.RotateSecondaryRayLocal(angle, axis);
            }
        }

        public void SetBodyUp(Vector3 up)
        {
            foreach(FullIKInfo info in infos)
            {
                info.solver.SetBodyUp(up);
            }
        }

        public void SetIdealHeight(float h)
        {
            foreach(FullIKInfo info in infos)
            {
                info.solver.SetIdealHeight(h);
            }
        }

        public void AddRotation(float globalAngle, Vector3 globalAxis, float localAngle, Vector3 localAxis)
        {
            foreach(FullIKInfo info in infos)
            {
                if (info.IsIKAvailable)
                    info.solver.RotateRays(globalAngle, globalAxis, localAngle, localAxis);
            }
        }

        public void CalculateRayLocalOffset(Vector3 worldPoint, Vector3 bodyUp)
        {
            foreach(FullIKInfo info in infos)
            {
                if(info.IsIKAvailable)
                {
                    info.solver.CalculateRayLocalOffset(worldPoint, bodyUp);
                }
            }    
        }

        /// <summary>
        /// Calculates center of mass, returns global point
        /// </summary>
        /// <returns>Calculated center of mass</returns>
        public Vector3 CalculateCenterOfMass()
        {
            if (infos.Length == 0)
                return Vector3.zero;

            Vector3 centerOfMass = Vector3.zero;

            float areaTotal = 0;
            FullIKInfo p1 = null, p2 = null;
            for(int i = 0; i < infos.Length; i++)
            {
                if (infos[i].IsIKAvailable)
                {
                    if(p1 == null)
                        p1 = infos[i];
                    else if(p2 == null)
                        p2 = infos[i];
                    else
                    {
                        FullIKInfo p3 = infos[i];
                        Vector3 edge1 = p3.TipProjectedOnRoot() - p1.TipProjectedOnRoot();
                        Vector3 edge2 = p3.TipProjectedOnRoot() - p2.TipProjectedOnRoot();

                        Vector3 cross = Vector3.Cross(edge1, edge2);
                        float area = cross.magnitude / 2f;

                        centerOfMass += area * (p1.TipProjectedOnRoot() + p2.TipProjectedOnRoot() + p3.TipProjectedOnRoot()) / 3f;

                        areaTotal += area;
                        p2 = p3;
                    }
                }
            }

            //Not enough points to calculate centroid
            if(Mathf.Approximately(areaTotal, 0f))
            {
                if(p2 != null)
                {
                    //Debug.Log("2 point normal");
                    //There is 2 points to calculate, then take middle point
                    centerOfMass = Vector3.Lerp(p2.TipProjectedOnRoot(), p1.TipProjectedOnRoot(), 0.5f);
                }
                else if(p1 != null) //This is a temporary solution
                {
                    //Debug.Log("1 point normal");
                    //There is only 1 point to calculate
                    centerOfMass = p1.GetPassiveRoot.position;
                }
            }
            else
            {
                centerOfMass /= areaTotal;
            }

            return centerOfMass;
        }

        /// <summary>
        /// Calculates center of mass of grounded fingers, returns global point
        /// </summary>
        /// <returns>Calculated center of mass</returns>
        public Vector3 CalculateGroundedCenterOfMass()
        {
            if (infos.Length == 0)
                return Vector3.zero;

            Vector3 centerOfMass = Vector3.zero;

            float areaTotal = 0;
            FullIKInfo p1 = null, p2 = null;
            for (int i = 0; i < infos.Length; i++)
            {
                if (infos[i].IsIKAvailable && infos[i].solver.onGround && !infos[i].solver.rayOverriden)
                {
                    if (p1 == null)
                        p1 = infos[i];
                    else if (p2 == null)
                        p2 = infos[i];
                    else
                    {
                        FullIKInfo p3 = infos[i];
                        Vector3 edge1 = p3.TipProjectedOnRoot() - p1.TipProjectedOnRoot();
                        Vector3 edge2 = p3.TipProjectedOnRoot() - p2.TipProjectedOnRoot();

                        Vector3 cross = Vector3.Cross(edge1, edge2);
                        float area = cross.magnitude / 2f;

                        centerOfMass += area * (p1.TipProjectedOnRoot() + p2.TipProjectedOnRoot() + p3.TipProjectedOnRoot()) / 3f;

                        areaTotal += area;
                        p2 = p3;
                    }
                }
            }

            //Not enough points to calculate centroid
            if (Mathf.Approximately(areaTotal, 0f))
            {
                if (p2 != null)
                {
                    //Debug.Log("2 point grounded");
                    //There is 2 points to calculate, then take middle point
                    centerOfMass = Vector3.Lerp(p2.TipProjectedOnRoot(), p1.TipProjectedOnRoot(), 0.5f);
                }
                else if(p1 != null) //This is a temporary solution
                {
                    //Debug.Log("1 point grounded");
                    //There is only 1 point to calculate
                    centerOfMass = p1.GetPassiveRoot.position;
                }
            }
            else
            {
                centerOfMass /= areaTotal;
            }

            return centerOfMass;
        }

        /// <summary>
        /// Calculates maximum height as if fingers are on ground
        /// </summary>
        /// <param name="upDir">Direction to calculate maximum extend</param>
        /// <returns></returns>
        public float MaxHeightRaw(Vector3 upDir)
        {
            float maxReach = float.MaxValue;
            foreach(FullIKInfo info in infos)
            {
                if (!info.solver.onGround)
                    continue;

                maxReach = Mathf.Min(maxReach, info.CalculateReachableRange(upDir));
            }

            return maxReach;
        }

        /// <summary>
        /// Calculates maximum height as if fingers are on ground
        /// </summary>
        /// <param name="pos">Position of the point</param>
        /// <param name="upDir">Direction to calculate maximum extend</param>
        /// <returns></returns>
        public float MaxHeight(Vector3 pos, Vector3 upDir)
        {
            return (pos + upDir * MaxHeightRaw(upDir)).y;
        }

        /// <summary>
        /// Average normal vector of all IK targets (not normalized).
        /// </summary>
        /// <param name="point">Helper point. Recommended to use center of the IKs</param>
        /// <param name="defaultUp">If IK not on ground, then use this vector as normal</param>
        /// <returns>Average normal vector (not normalized)</returns>
        public Vector3 AverageTargetNormalPythagor(Vector3 point, Vector3 defaultUp)
        {
            Vector3 normal = Vector3.zero;
            foreach (FullIKInfo info in infos)
            {
                normal += info.CalculateNormal((point - info.GetPassiveRoot.position) * info.solver.RayLength);
            }

            return normal;
        }

        /// <summary>
        /// Average position of all IK targets.
        /// </summary>
        /// <returns>Average position (world coordinate)</returns>
        public Vector3 AverageTargetPosition()
        {
            //If there is no fingers, return hand pivot position
            if (infos.Length == 0)
                return transform.position;

            //Calculate and return fingers average target position
            Vector3 total = Vector3.zero;
            foreach (FullIKInfo info in infos)
            {
                total += info.solver.worldPlacePoint;
            }
            return total / infos.Length;
        }

        /// <summary>
        /// Syncs joints rotation with ik rotation
        /// </summary>
        public void SyncJointsWithIK()
        {
            foreach (FullIKInfo info in infos)
            {
                info.SyncJoints();
            }
        }

        /// <summary>
        /// Returns true if any finger is on ground
        /// </summary>
        /// <returns></returns>
        public bool OnGround()
        {
            //If at least one finger is on ground, return true
            bool onGround = false;
            foreach (FullIKInfo info in infos)
            {
                if (info.solver.onGround && Vector3.Distance(info.solver.worldPlacePoint, info.GetActiveRoot.position) <= info.solver.RayLength)
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
        public bool CanMove()
        {
            //If at least one finger is stationary, return true
            bool canMove = false;
            foreach (FullIKInfo info in infos)
            {
                if (!info.solver.isMoving)
                {
                    canMove = true;
                    break;
                }
            }

            return canMove;
        }
    }
}
