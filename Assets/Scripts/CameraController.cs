using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Simple third person camera controller
[RequireComponent(typeof(Camera))]
public class CameraController : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 globalOffset, localOffset;
    [SerializeField] private float distance = 5f;
    [SerializeField] private float sensitivity = 3f;
    [SerializeField] private float smoothTime = 0.3f;

    private Vector3 velocity = Vector3.zero;

    private Camera cam;
    private float raySphereRadius;

    private const float MIN_X_ANGLE = -85f;
    private const float MAX_X_ANGLE = 85f;

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
    }

    private void LateUpdate()
    {
        Vector3 localEuler = transform.localEulerAngles;

        localEuler.y += -Input.GetAxisRaw("Mouse X") * sensitivity;
        localEuler.x += Input.GetAxisRaw("Mouse Y") * sensitivity;

        //Camera zoom in and out
        distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel") * 5f, 1f, 10f);

        //Camera position
        //transform.position = Vector3.Lerp(transform.position, CheckCollision(CalculatePosition()), Time.deltaTime * smoothTime);
        transform.position = Vector3.Slerp(transform.position, CheckCollision(CalculatePosition()), Time.deltaTime * smoothTime);
        //transform.position = Vector3.SmoothDamp(transform.position, CheckCollision(CalculatePosition()), ref velocity, smoothTime);

        //Angle limit calculations
        Vector3 newLocalEuler = localEuler;

        newLocalEuler.z = 0f;
        if (newLocalEuler.x > 180f)
            newLocalEuler.x -= 360f;

        newLocalEuler.x = Mathf.Clamp(newLocalEuler.x, MIN_X_ANGLE, MAX_X_ANGLE);

        transform.localEulerAngles = newLocalEuler;
    }

    //Position calculations
    private Vector3 CalculatePosition()
    {
        return CalculateRawPosition() - transform.forward * distance;
    }

    //Focus point calculations
    private Vector3 CalculateRawPosition()
    {
        return target.position + globalOffset + transform.TransformVector(localOffset);
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
