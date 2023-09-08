using UnityEngine;
using UnityEngine.Events;

namespace BIK
{
    /// <summary>
    /// Sends collision messages as unity events, that way objects which doesn't have a rigidbody component can notified by collision events
    /// </summary>
    public class CollisionDetector : MonoBehaviour
    {
        public bool isEnabled = false;

        public UnityEvent onCollisionEnter = new UnityEvent();
        public UnityEvent onCollisionExit = new UnityEvent();

        private void OnCollisionEnter(Collision collision)
        {
            if(isEnabled)
                onCollisionEnter.Invoke();
        }

        private void OnCollisionStay(Collision collision)
        {
            if(isEnabled)
                onCollisionEnter.Invoke();
        }

        private void OnCollisionExit(Collision collision)
        {
            if(isEnabled)
                onCollisionExit.Invoke();
        }

        public void AddMethodToEnter(UnityAction action)
        {
            onCollisionEnter.AddListener(action);
        }

        public void AddMethodToExit(UnityAction action)
        {
            onCollisionExit.AddListener(action);
        }
    }
}
