using UnityEngine;

namespace BIK
{
    public class StickySurface : MonoBehaviour
    {
        private Transform stickyParent;
        private GameObject stickyLinePrefab;

        private void Awake()
        {
            stickyParent = new GameObject("StickyLines").transform;
            stickyParent.SetParent(transform);

            stickyLinePrefab = Resources.Load<GameObject>("StickyLine");
        }

        private void Update()
        {
            foreach(StickyLineTracker tracker in stickyParent.GetComponentsInChildren<StickyLineTracker>())
            {
                tracker.CustomUpdate();
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            if(collision.gameObject.TryGetComponent(out Rigidbody rb))
            {
                Joint[] joints = GetComponents<Joint>();
                foreach(Joint joint in joints)
                {
                    if (rb == joint.connectedBody) //Already connected
                        return;
                }

                /*if(rb.TryGetComponent(out Joint j)) //Body part
                {
                    SpringJoint spring = gameObject.AddComponent<SpringJoint>();
                    spring.spring = 25f;
                    spring.damper = 20f;
                    spring.breakForce = 90f;
                    spring.enablePreprocessing = false;
                    spring.connectedBody = rb;
                }
                else //Normal object
                {
                    FixedJoint fJoint = gameObject.AddComponent<FixedJoint>();
                    fJoint.breakForce = 300f;
                    fJoint.connectedBody = rb;
                }*/

                SpringJoint spring = gameObject.AddComponent<SpringJoint>();
                spring.spring = 25f;
                spring.damper = 10f;
                spring.breakForce = 70f;
                spring.enableCollision = true;
                spring.enablePreprocessing = false;
                spring.autoConfigureConnectedAnchor = false;
                spring.connectedAnchor = rb.transform.InverseTransformPoint(collision.GetContact(0).point);
                spring.connectedBody = rb;

                rb.velocity *= 0;
                rb.angularVelocity *= 0;

                GameObject stickyLine = Instantiate(stickyLinePrefab,
                    collision.GetContact(0).point,
                    Quaternion.LookRotation(rb.position - transform.position),
                    stickyParent);
                stickyLine.GetComponent<StickyLineTracker>().Initialize(rb.transform, spring);
            }
        }
    }
}
