using UnityEngine;

public class IKGroupHolder : MonoBehaviour
{
    [System.Serializable]
    public struct Group
    {
#if UNITY_EDITOR
        [Tooltip("Editor only"), SerializeField] private string groupName;
#endif
        public TargetIKSolver[] iks;
    }

    [SerializeField] private Group[] solverGroups;
    [SerializeField] private Transform helperObject; //Used in normal calculation

    /// <summary>
    /// All IK group availabilities.
    /// </summary>
    /// <returns>returns at least one false if any IK solver target is moving</returns>
    public bool[] GetGroupAvailabilities()
    {
        bool[] results = new bool[solverGroups.Length];

        for(int i = 0; i < results.Length; i++)
        {
            results[i] = false;
            foreach(TargetIKSolver solver in solverGroups[i].iks)
            {
                if (!solver.isMoving)
                {
                    results[i] = true;
                    break;
                }
            }
        }

        return results;
    }

    /// <summary>
    /// Average position of all IK targets.
    /// </summary>
    /// <returns>Average position</returns>
    public Vector3 AverageTargetPos()
    {
        Vector3 pos = Vector3.zero;
        int total = 0;

        foreach(Group g in solverGroups)
        {
            foreach (TargetIKSolver solver in g.iks)
                pos += solver.worldPlacePoint;

            total += g.iks.Length;
        }

        return pos / total;
    }

    public Vector3 LowestPos()
    {
        Vector3 pos = new Vector3(0f, float.MaxValue, 0f);

        foreach (Group g in solverGroups)
        {
            foreach (TargetIKSolver solver in g.iks)
            {
                if(pos.y > solver.worldPlacePoint.y)
                    pos = solver.worldPlacePoint;
            }
        }

        return pos;
    }

    /// <summary>
    /// Average normal vector of all IK targets.
    /// </summary>
    /// <returns>Average normal vector</returns>
    public Vector3 AverageTargetNormal()
    {
        ///Calculation done with repetitive triangle surface normal calculation
        ///Lets say we have ABC triangle, A is our average position and B is our helper object
        ///We will iterate C point for every IK solver and adding to the normal vector

        Vector3 avg = AverageTargetPos();
        Vector3 normal = Vector3.zero;

        foreach (Group g in solverGroups)
        {
            foreach (TargetIKSolver solver in g.iks)
            {
                Vector3 n = Vector3.Cross((solver.worldPlacePoint - avg).normalized, (solver.worldPlacePoint - helperObject.position).normalized);
                if (Vector3.Dot(n, solver.rayDir) > 0f)
                    n *= -1; //Correcting normal vector direction
                Debug.DrawRay(solver.worldPlacePoint, n, Color.blue);
                Debug.DrawLine(solver.worldPlacePoint, avg, Color.red);
                Debug.DrawLine(solver.worldPlacePoint, helperObject.position, Color.red);
                normal += n;
            }
        }

        return normal;
    }
}
