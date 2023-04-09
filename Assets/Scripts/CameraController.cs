using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Simple third person camera controller
[RequireComponent(typeof(Camera))]
public class CameraController : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 globalOffset, localOffset;
    [SerializeField] private Transform lookTarget;
    [SerializeField] private Vector3 lookGlobalOffset, lookLocalOffset;
    [SerializeField] private float distance = 5f;
    [SerializeField] private float sensitivity = 3f;
    [SerializeField] private float smoothTime = 0.3f;

    //private Vector3 velocity = Vector3.zero; //Uncomment when using smoothstep
    private Vector3 direction;

    private Camera cam;
    private float raySphereRadius;

    private const float MIN_X_EULER = -75f;
    private const float MAX_X_EULER = 70f;

    public static CameraController instance;
    private void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }
        else
            Destroy(gameObject);

        cam = GetComponent<Camera>();
    }

    private void Start()
    {
        UpdateRaySphereRadius();

        direction = (transform.position - CalculateRawPosition()).normalized;
    }

    private void LateUpdate()
    {
        float inputX = Input.GetAxisRaw("Mouse X") * sensitivity;
        float inputY = Input.GetAxisRaw("Mouse Y") * sensitivity;

        //Angle limit calculations
        Vector3 flatYSurface = -transform.forward;
        flatYSurface.y = 0f;
        float signedAngle = Vector3.SignedAngle(flatYSurface, direction, transform.right);

        inputY = Mathf.Clamp(signedAngle + inputY, MIN_X_EULER, MAX_X_EULER) - signedAngle;

        //Set direction
        direction = Quaternion.AngleAxis(inputX, Vector3.up) * Quaternion.AngleAxis(inputY, transform.right) * direction;

        //Camera zoom in and out
        distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel") * 5f, 1f, 10f);

        //Camera position
        //transform.position = Vector3.Lerp(transform.position, CheckCollision(CalculatePosition()), Time.deltaTime * smoothTime);
        transform.position = Vector3.Slerp(transform.position, CheckCollision(CalculatePosition()), Time.deltaTime * smoothTime);
        //transform.position = Vector3.SmoothDamp(transform.position, CheckCollision(CalculatePosition()), ref velocity, smoothTime);

        //Camera rotation
        transform.LookAt(CalculateLookPosition());
    }

    //Position calculations
    private Vector3 CalculatePosition()
    {
        return CalculateRawPosition() + direction * distance;
    }

    //Target point calculations
    private Vector3 CalculateRawPosition()
    {
        return target.position + globalOffset + transform.TransformVector(localOffset);
    }

    //Focus point calculations
    private Vector3 CalculateLookPosition()
    {
        return lookTarget.position + lookGlobalOffset + lookTarget.TransformVector(lookLocalOffset);
    }

    //Updates ray sphere radius (update the radius if Camera's near clip plane, field of view or aspect is changed)
    private void UpdateRaySphereRadius()
    {
        Vector3 forwardL = transform.forward * cam.nearClipPlane;

        float cameraHeight = Mathf.Tan(cam.fieldOfView * Mathf.Deg2Rad * 0.5f);
        Vector3 upL = transform.up * Mathf.Tan(cameraHeight) * cam.nearClipPlane;

        Vector3 rightL = transform.right * cameraHeight * cam.aspect * cam.nearClipPlane;

        raySphereRadius = (forwardL + upL + rightL).magnitude * 0.5f;
    }

    //Check for any camera collision
    private Vector3 CheckCollision(Vector3 restPos)
    {
        Vector3 rawPos = CalculateRawPosition();
        Vector3 direction = restPos - rawPos;
        RaycastHit hitInfo;

        if (Physics.SphereCast(rawPos, raySphereRadius, direction, out hitInfo, distance))
        {
            restPos = rawPos + direction.normalized * hitInfo.distance;
        }

        return restPos;
    }
}
