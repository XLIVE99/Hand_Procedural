using UnityEngine;

namespace HandWar
{
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
            if (isDebug)
                Debug.Log("errorLast: " + store.errorLast + " error: " + error + "\nvalueLast: " + store.valueLast + " value: " + current);

            float d = 0;
            if (store.derivativeInitialized)
            {
                if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                {
                    float errorRateOfChange = (error - store.errorLast) / dt;
                    store.errorLast = error;

                    d = (error - store.errorLast) / dt;
                }
                else
                {
                    float valueRateOfChange = (current - store.valueLast) / dt;
                    store.valueLast = current;

                    d = -valueRateOfChange;
                }
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
            if (isDebug)
                Debug.Log("errorLast: " + store.errorLast + " error: " + error + "\nvalueLast: " + store.valueLast + " value: " + current);

            float d = 0;
            if (store.derivativeInitialized)
            {
                if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                {
                    float errorRateOfChange = AngleDifference(error, store.errorLast) / dt;
                    store.errorLast = error;

                    d = errorRateOfChange;
                }
                else
                {
                    float valueRateOfChange = AngleDifference(current, store.valueLast) / dt;
                    store.valueLast = current;

                    d = -valueRateOfChange;
                }
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

            Vector3 d = Vector3.zero;
            if (store.derivativeInitialized)
            {
                if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                {
                    Vector3 errorRateOfChange = (error - store.errorLast) / dt;
                    store.errorLast = error;

                    d = errorRateOfChange;
                }
                else
                {
                    Vector3 valueRateOfChange = (current - store.valueLast) / dt;
                    store.valueLast = current;

                    d = -valueRateOfChange;
                }
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

        public Quaternion UpdatePID(float dt, Quaternion current, Quaternion goal, PIDStore<Quaternion> store)
        {
            //if in normal math c = a - b then in quaternion c = inverse(b) * a
            Quaternion error = Quaternion.Inverse(current) * goal;
            error.ToAngleAxis(out float errorAngle, out Vector3 errorAxis);

            //P term
            //Quaternion p = Quaternion.AngleAxis(errorAngle * (proportionalGain / Mathf.Rad2Deg), errorAxis);

            //D term
            if (isDebug)
                Debug.Log("errorLast: " + store.errorLast + " error: " + error + "\nvalueLast: " + store.valueLast + " value: " + current);

            Quaternion d = Quaternion.identity;
            if (store.derivativeInitialized)
            {
                if (derivativeMeasurement == DerivativeMeasurement.ErrorRateOfChange)
                {
                    Quaternion diffError = Quaternion.Inverse(store.errorLast) * error;
                    diffError.ToAngleAxis(out float diffAngle, out Vector3 diffAxis);

                    Quaternion errorRateOfChange = Quaternion.AngleAxis(diffAngle / dt, diffAxis);
                    store.errorLast = error;

                    d = errorRateOfChange;
                }
                else
                {
                    Quaternion diffValue = Quaternion.Inverse(store.valueLast) * current;
                    diffValue.ToAngleAxis(out float diffAngle, out Vector3 diffAxis);

                    Quaternion valueRateOfChange = Quaternion.AngleAxis(diffAngle / dt, diffAxis);
                    store.valueLast = current;

                    d = Quaternion.Inverse(valueRateOfChange);
                }
            }
            else
                store.derivativeInitialized = true;

            float dAngle = 0f;
            Vector3 dAxis = Vector3.one;
            if(d != Quaternion.identity)
                d.ToAngleAxis(out dAngle, out dAxis);

            //d = Quaternion.AngleAxis(dAngle * (derivativeGain / Mathf.Rad2Deg), dAxis);

            //Calculate I term
            float integMax = 0f;

            //p.ToAngleAxis(out float pAngle, out Vector3 pAxis);
            float pMagnitude = errorAngle;

            if (integralSaturation > pMagnitude)
                integMax = integralSaturation - pMagnitude;

            store.integrationStored.ToAngleAxis(out float integrationStoreAngle, out Vector3 integrationStoreAxis);
            integrationStoreAngle = Mathf.Min(integMax, integrationStoreAngle);

            //store.integrationStored = Vector3.ClampMagnitude(store.integrationStored + (error * dt), integMax);
            store.integrationStored = Quaternion.RotateTowards(Quaternion.AngleAxis(integrationStoreAngle, integrationStoreAxis),
                store.integrationStored * Quaternion.AngleAxis(errorAngle * dt, errorAxis), integMax);

            float iAngle = 0f;
            Vector3 iAxis = Vector3.one;
            if(store.integrationStored != Quaternion.identity)
                store.integrationStored.ToAngleAxis(out iAngle, out iAxis);

            Vector3 finalVector = (errorAxis * errorAngle * proportionalGain * Mathf.Deg2Rad) //p value
                + (dAxis * dAngle * derivativeGain * Mathf.Deg2Rad) //d value
                + (iAxis * iAngle * integralGain * Mathf.Deg2Rad); //i value
            return Quaternion.AngleAxis(Mathf.Clamp(finalVector.magnitude, outputRange.x, outputRange.y), finalVector);
        }

        //Range: [-180, 180]
        private float AngleDifference(float a, float b)
        {
            return (a - b + 540) % 360 - 180;
        }
    }
}