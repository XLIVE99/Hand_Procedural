using UnityEngine;

public class TargetIKSolver : MonoBehaviour
{
    [SerializeField] private Transform rayPoint;
    [SerializeField] private float rayLength = 3f;
    [SerializeField] private Transform target;
    [SerializeField] private float moveDistance;
    [SerializeField] private float moveSpeed = 2f;
    [SerializeField] private float rayRadius = 0.2f;

    private Vector3 oldPos;
    public Vector3 worldPlacePoint { get; private set; }
    private float lerp;
    public bool isMoving { get; private set; } = false;

    public Vector3 rayDir { get { return -rayPoint.up; } }

    private const float MOVE_UP_LENGTH = 0.4f;

    private void Start()
    {
        CheckPos(true);
        lerp = 1f; //Instantly place target to the worldPlacePoint
    }

    private void Update()
    {
        PlaceTarget(worldPlacePoint);
    }

    private void FixedUpdate()
    {
        CheckPos();
    }

    private void CheckPos(bool force = false)
    {
        if(Physics.SphereCast(rayPoint.position, rayRadius, -rayPoint.up, out RaycastHit hitInfo, rayLength, LayerMask.GetMask("Terrain")))
        {
            if(force || Vector3.Distance(worldPlacePoint, hitInfo.point) >= moveDistance)
            {
                oldPos = worldPlacePoint;
                worldPlacePoint = hitInfo.point;
                isMoving = true;
                lerp = 0;
            }
            else if(isMoving)
            {
                worldPlacePoint = hitInfo.point;
            }
        }
    }

    private void PlaceTarget(Vector3 worldPos)
    {
        target.position = Vector3.Lerp(oldPos, worldPos, lerp) + Vector3.up * MOVE_UP_LENGTH * Mathf.Sin(lerp * Mathf.PI);

        if(isMoving)
        {
            lerp = Mathf.MoveTowards(lerp, 1f, Time.deltaTime * moveSpeed);
            if (lerp >= 1f)
            {
                lerp = 1f;
                isMoving = false;
            }
        }
    }
}
