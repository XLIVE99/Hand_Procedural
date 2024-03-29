using UnityEngine;
using UnityEngine.Animations.Rigging;

namespace BIK.SemiActive
{
    public class SemiIKHolder : MonoBehaviour
    {
        [SerializeField] private SemiIKInfo[] infos;
        [SerializeField] private RigBuilder rigBuilder;

        public void Start()
        {
            foreach (SemiIKInfo info in infos)
                info.Initialize();
        }

        public void FireSegment(SemiIKInfo finger)
        {
            finger.FireSegment();

            //In order to build properly, animator must have default animation
            //Bones must be in default rotation (or position if you modifying bones position with IK) in that default animation
            rigBuilder.Build();
        }

        /// <summary>
        /// Returns active fingers
        /// </summary>
        /// <returns></returns>
        public int GetActiveFingers()
        {
            int i = 0;
            foreach(SemiIKInfo info in infos)
            {
                if (info.IsIKAvailable)
                    i++;
            }
            return i;
        }

        /// <summary>
        /// Returns a ready to fire finger
        /// </summary>
        /// <returns></returns>
        public SemiIKInfo GetAvailableIK()
        {
            foreach(SemiIKInfo info in infos)
            {
                if(info.IsIKAvailable)
                {
                    return info;
                }
            }

            return null;
        }

        public Vector3 CenterOfMass()
        {
            Vector3 centerOfMass = Vector3.zero;

            if (infos.Length == 0)
                return centerOfMass;

            //Calculate mid point based on all finger root's positions
            foreach (SemiIKInfo info in infos)
            {
                centerOfMass += info.root.position;
            }

            //Take the average of the fingers positions
            centerOfMass /= infos.Length;

            return centerOfMass;
        }

        /// <summary>
        /// Calculates maximum height as if fingers are on ground
        /// </summary>
        /// <param name="upDir">Direction to calculate maximum extend</param>
        /// <returns></returns>
        public float MaxHeight(Vector3 pos, Vector3 upDir)
        {
            float maxReach = float.MaxValue;
            foreach(SemiIKInfo info in infos)
            {
                if (!info.solver.onGround)
                    continue;

                maxReach = Mathf.Min(maxReach, info.CalculateReachableRange(upDir));
            }

            return (pos + upDir * maxReach).y;
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
            foreach (SemiIKInfo info in infos)
            {
                normal += info.CalculateNormal((point - info.root.position) * info.solver.RayLength);
            }

            return normal;
        }

        public Vector3 AverageTargetNormalBasic()
        {
            Vector3 normal = Vector3.zero;
            foreach (SemiIKInfo info in infos)
            {
                normal += info.BasicNormal();
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
            foreach (SemiIKInfo info in infos)
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
            foreach (SemiIKInfo info in infos)
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
            foreach (SemiIKInfo info in infos)
            {
                if (info.solver.onGround && Vector3.Distance(info.solver.worldPlacePoint, info.rootActive.position) <= info.solver.RayLength)
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
            foreach (SemiIKInfo info in infos)
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
