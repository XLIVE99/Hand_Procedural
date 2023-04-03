using UnityEngine;

public class TestHoverHeight : MonoBehaviour
{
    [SerializeField] private Transform upObject;
    [SerializeField] private IkMimic f1, f2;

    [System.Serializable]
    public struct IkMimic
    {
        public Transform root;
        public Transform ikTarget;
        public float extent;
    }

    private void OnDrawGizmos()
    {
        if (f1.ikTarget == null || f1.root == null ||
            f2.root == null || f2.ikTarget == null)
            return;

        Gizmos.color = Color.black;
        Gizmos.DrawLine(f1.root.position, f2.root.position);
        Gizmos.DrawSphere(f1.root.position, 0.05f);

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(f1.ikTarget.position, f1.extent);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(f2.ikTarget.position, f2.extent);
    }

    private void Update()
    {
        Method2();

    }

    //Requires at least 2 fingers
    private void Method1()
    {
        float maxReach = float.MaxValue;
        //Vector3 avgNormal = AverageTargetNormal().normalized;
        Vector3 diffRoot = f2.root.position - f1.root.position;
        Debug.DrawRay(f1.root.position, diffRoot, Color.black);
        IkMimic domHolder = f1, subHolder = f2;
        Vector3 lastPoint = f1.ikTarget.position + diffRoot;


        Vector3 upDir = -Vector3.Cross(diffRoot, Vector3.forward).normalized;
        Debug.DrawRay(f1.root.position + diffRoot * 0.5f, upDir, Color.green);
        float currentDomDist = (domHolder.root.position - domHolder.ikTarget.position).magnitude;

        //First calculate dominant max distance (Law of sines)
        Vector3 subToLast = lastPoint - subHolder.ikTarget.position;
        float sideC = Vector3.Distance(lastPoint, subHolder.ikTarget.position);
        float maxExtentUp = 0;
        if (sideC <= subHolder.extent)
        {
            float angleB = Vector3.Angle(-subToLast, upDir);
            if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
            {
                //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                //Just take extent
                if (angleB > 90f)
                    maxExtentUp = subHolder.extent - sideC;
                //If B equals to 0 degree then up direction and subToLast vectors are in opposite direction
                //Reverse the extent
                else
                    maxExtentUp = -sideC;
                Debug.Log("AngleB: " + angleB + " sideC: " + sideC);
            }
            else
            {
                float angleC = Mathf.Asin((sideC * Mathf.Sin(angleB * Mathf.Deg2Rad)) / subHolder.extent) * Mathf.Rad2Deg;
                float angleA = 180f - angleB - angleC;
                maxExtentUp = (subHolder.extent * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                Debug.Log("AngleA: " + angleA + " B: " + angleB + " C: " + angleC);
            }
            Debug.Log("MaxExtentUp: " + maxExtentUp);
        }

        Debug.DrawRay(subHolder.ikTarget.position, subToLast, Color.yellow);

        float maxReachable = Mathf.Min(maxExtentUp, domHolder.extent - currentDomDist, subHolder.extent);
        Debug.DrawRay(subHolder.ikTarget.position, upDir * maxReachable, Color.gray);

        Debug.DrawLine(domHolder.ikTarget.position + upDir * maxReachable, subHolder.ikTarget.position + upDir * maxReachable, Color.cyan);
        maxReach = Mathf.Min(maxReach, maxReachable);
    }

    //Can work with only a finger but requires global up direction
    private void Method2()
    {
        float maxReach = float.MaxValue;
        //Vector3 avgNormal = AverageTargetNormal().normalized;
        Vector3 diffRoot = f1.root.position - f1.ikTarget.position;

        Vector3 upDir = upObject.up;
        Debug.DrawRay(f1.root.position + diffRoot * 0.5f, upDir, Color.green);
        float currentDist = (f1.root.position - f1.ikTarget.position).magnitude;

        //First calculate dominant max distance (Law of sines)
        float sideC = diffRoot.magnitude;
        float maxExtentUp = 0;
        if (sideC <= f1.extent)
        {
            float angleB = Vector3.Angle(-diffRoot, upDir);
            if (Mathf.Approximately(angleB, 180f) || Mathf.Approximately(angleB, 0f))
            {
                //If B equals to 180 degree then up direction and subToLast vectors are in same direction
                //Just take extent
                if (angleB > 90f)
                    maxExtentUp = f1.extent - sideC;
                //If B equals to 0 degree then up direction and subToLast vectors are in opposite direction
                //Reverse the extent
                else
                    maxExtentUp = -sideC;
                Debug.Log("AngleB: " + angleB + " sideC: " + sideC);
            }
            else
            {
                float angleC = Mathf.Asin((sideC * Mathf.Sin(angleB * Mathf.Deg2Rad)) / f1.extent) * Mathf.Rad2Deg;
                float angleA = 180f - angleB - angleC;
                maxExtentUp = (f1.extent * Mathf.Sin(angleA * Mathf.Deg2Rad)) / Mathf.Sin(angleB * Mathf.Deg2Rad);
                Debug.Log("AngleA: " + angleA + " B: " + angleB + " C: " + angleC);
            }
            Debug.Log("MaxExtentUp: " + maxExtentUp);
        }

        Debug.DrawRay(f1.ikTarget.position, diffRoot, Color.yellow);

        float maxReachable = Mathf.Min(maxExtentUp, f1.extent - currentDist);
        Debug.DrawRay(f1.ikTarget.position, upDir * maxReachable, Color.gray);

        Debug.DrawLine(f1.ikTarget.position + upDir * maxReachable, f1.ikTarget.position + upDir * maxReachable, Color.cyan);
        maxReach = Mathf.Min(maxReach, maxReachable);
    }
}
