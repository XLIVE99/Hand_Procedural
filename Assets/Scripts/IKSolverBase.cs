using UnityEngine;
using UnityEngine.Events;

namespace BIK
{
    /// <summary>
    /// IK Solver base class
    /// </summary>
    public class IKSolverBase : MonoBehaviour
    {
        [SerializeField, Header("--- Ray ---")] protected Transform rayPoint; //Spherecast point
        [SerializeField] protected float rayLength = 3f; //Ray length
        public float RayLength => rayLength;
        [SerializeField] protected float rayRadius = 0.2f; //Spherecast radius

        [SerializeField, Header("--- Other ---")] protected float moveSpeed = 2f; //Step speed

        public Vector3 worldPlacePoint { get; protected set; } //IK target position
        public RaycastHit lastHit { get; protected set; } //Last successfull ray hit info

        public bool isMoving { get; protected set; } = false; //Is solver stepping
        public bool onGround { get; protected set; } = false; //Is ray hitting ground

        #region Step Restrictions (NOT USED)

        /// <summary>
        /// Not used currently. This section can be used to restrict stepping algorithm even more to make it more realistic.
        /// </summary>

        /*[HideInInspector] public UnityEvent<IKSolverBase, bool> OnMovingChange = new UnityEvent<IKSolverBase, bool>();
        protected bool _canStep = true;
        public bool CanStep { get { return _canStep; } set { _canStep = value; CanStepChanged(); } }
        protected virtual void CanStepChanged()
        {
            if (isMoving && !CanStep)
                isMoving = false;
        }*/
        #endregion

        public void ChangeRayLength(float rl)
        {
            rayLength = rl;

            //Prevent unnecessary calculation to gain performance
            if (rl > 0.01f != enabled)
                enabled = rl > 0.01f;
        }

        public virtual void ChangeMoveSpeed(float m)
        {
            moveSpeed = m;
        }

    }
}
