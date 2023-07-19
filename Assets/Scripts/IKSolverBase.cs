using UnityEngine;

namespace HandWar
{
    public class IKSolverBase : MonoBehaviour
    {
        public bool isMoving { get; protected set; } = false;
        public bool onGround { get; protected set; } = false;
    }
}
