using UnityEngine;

public class PIDController : MonoBehaviour
{
    [SerializeField] private float proportionalGain; //Spring
    [SerializeField] private float integralGain;
    [SerializeField] private float integralSaturation = 5f;
    [SerializeField] private float derivativeGain; //Dampener
    [SerializeField] private Vector2 outputRange;

    [SerializeField] private bool isDebug;

    private enum DerivativeMeasurement
    {
        Velocity,
        ErrorRateOfChange
    }
    [SerializeField] private DerivativeMeasurement derivativeMeasurement;

    public float UpdatePID(float dt, float current, float goal, PIDStore<float> store)
    {
        float error = goal - current;

        //P term
        float p = proportionalGain * error;

        //D term
        float errorRateOfChange = (error - store.errorLast) / dt;
        store.errorLast = error;

        float valueRateOfChange = (current - store.valueLast) / dt;
        store.valueLast = current;

        float d = 0;
        if (store.derivativeInitialized)
        {
            if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                d = errorRateOfChange;
            else
                d = -valueRateOfChange;
        }
        else
            store.derivativeInitialized = true;

        d *= derivativeGain;

        //Calculate I term

        float integMax = 0f;
        float integMin = 0f;

        if (integralSaturation > p)
            integMax = integralSaturation - p;

        if (-integralSaturation < p)
            integMin = -integralSaturation - p;

        store.integrationStored = Mathf.Clamp(store.integrationStored + (error * dt), integMin, integMax);
        float i = store.integrationStored * integralGain;

        return Mathf.Clamp(p + i + d, outputRange.x, outputRange.y);
    }

    public float UpdateAnglePID(float dt, float current, float goal, PIDStore<float> store)
    {
        float error = AngleDifference(goal, current);

        //P term
        float p = proportionalGain * error;

        //D term
        float errorRateOfChange = AngleDifference(error, store.errorLast) / dt;
        store.errorLast = error;

        float valueRateOfChange = AngleDifference(current, store.valueLast) / dt;
        store.valueLast = current;

        float d = 0;
        if (store.derivativeInitialized)
        {
            if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                d = errorRateOfChange;
            else
                d = -valueRateOfChange;
        }
        else
            store.derivativeInitialized = true;

        d *= derivativeGain;

        //Calculate I term
        float integMax = 0f;
        float integMin = 0f;

        if (integralSaturation > p)
            integMax = integralSaturation - p;

        if (-integralSaturation < p)
            integMin = -integralSaturation - p;

        store.integrationStored = Mathf.Clamp(store.integrationStored + (error * dt), integMin, integMax);
        float i = store.integrationStored * integralGain;

        return Mathf.Clamp(p + i + d, outputRange.x, outputRange.y);
    }

    public Vector3 UpdatePID(float dt, Vector3 current, Vector3 goal, PIDStore<Vector3> store)
    {
        Vector3 error = goal - current;

        //P term
        Vector3 p = proportionalGain * error;

        //D term
        if (isDebug)
            Debug.Log("errorLast: " + store.errorLast + " error: " + error + "\nvalueLast: " + store.valueLast + " value: " + current);
        Vector3 errorRateOfChange = (error - store.errorLast) / dt;
        store.errorLast = error;

        Vector3 valueRateOfChange = (current - store.valueLast) / dt;
        store.valueLast = current;


        Vector3 d = Vector3.zero;
        if (store.derivativeInitialized)
        {
            if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                d = errorRateOfChange;
            else
                d = -valueRateOfChange;
        }
        else
            store.derivativeInitialized = true;

        d *= derivativeGain;

        //Calculate I term
        float integMax = 0f;

        float pMagnitude = p.magnitude;

        if (integralSaturation > pMagnitude)
            integMax = integralSaturation - pMagnitude;

        store.integrationStored = Vector3.ClampMagnitude(store.integrationStored + (error * dt), integMax);
        Vector3 i = store.integrationStored * integralGain;

        return Vector3.ClampMagnitude(p + i + d, Mathf.Max(outputRange.x, outputRange.y));
    }

    //Range: [-180, 180]
    private float AngleDifference(float a, float b)
    {
        return (a - b + 540) % 360 - 180;
    }
}
