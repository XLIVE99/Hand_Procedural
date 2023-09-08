using UnityEngine;

namespace BIK
{
    [RequireComponent(typeof(LineRenderer))]
    public class StickyLineTracker : MonoBehaviour
    {
        [SerializeField] private ParticleSystem splitParticle;

        private LineRenderer lr;
        private Transform trackObject;
        private Joint checkJoint;

        private void Awake()
        {
            lr = GetComponent<LineRenderer>();
        }

        public void Initialize(Transform trackObj, Joint j)
        {
            trackObject = trackObj;
            checkJoint = j;
        }

        public void CustomUpdate()
        {
            if(checkJoint == null)
            {
                //Set and play split particle
                Vector3 midPoint = transform.position + transform.TransformDirection(lr.GetPosition(1)) * 0.5f;
                float dist = lr.GetPosition(1).magnitude;
                splitParticle.transform.position = midPoint;
                splitParticle.transform.rotation = Quaternion.LookRotation(trackObject.position - transform.position);

                ParticleSystem.ShapeModule shape = splitParticle.shape;
                shape.radius = dist - 0.2f; //Restrict emission area

                splitParticle.transform.SetParent(null, true); //Prevent destroy with this object
                splitParticle.Play();

                //Destroy line
                Destroy(gameObject);
            }
            else
            {
                Vector3 worldPos = trackObject.TransformPoint(checkJoint.connectedAnchor);
                Vector3 dir = (worldPos - transform.position).normalized; //Direction to stick point (used to offset a bit towards sticked object)
                lr.SetPosition(1, transform.InverseTransformPoint(worldPos + dir * 0.1f));
            }
        }
    }
}
