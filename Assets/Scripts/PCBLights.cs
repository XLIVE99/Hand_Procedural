using UnityEngine;

namespace HandWar
{
    public class PCBLights : MonoBehaviour
    {
        [System.Serializable]
        public struct FingerInfo
        {
            public IKSolverBase solver;
            public GameObject groundLed;
            public GameObject moveLed;
        }

        [SerializeField] private FingerInfo[] fingers;

        private void Update()
        {
            foreach(FingerInfo finger in fingers)
            {
                SetActiveToBool(finger.groundLed, finger.solver.onGround);

                SetActiveToBool(finger.moveLed, !finger.solver.isMoving);
            }
        }

        private void SetActiveToBool(GameObject g, bool b)
        {
            if (g.activeSelf != b)
                g.SetActive(b);
        }
    }
}
