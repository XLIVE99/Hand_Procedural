using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BIK
{
    public class TestQuaternionPID : MonoBehaviour
    {
        [SerializeField] private Transform refObj;
        [SerializeField] private PIDController pid;

        PIDController.PIDStore<Quaternion> rotStore;

        Vector3 befValue;

        // Start is called before the first frame update
        void Start()
        {
            rotStore = new PIDController.PIDStore<Quaternion>();
        }

        // Update is called once per frame
        void Update()
        {
            Quaternion pidQuaternion = pid.UpdatePID(Time.deltaTime, transform.rotation, refObj.rotation, rotStore);
            transform.rotation *= pidQuaternion;
        }
    }
}
