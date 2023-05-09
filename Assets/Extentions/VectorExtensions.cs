using UnityEngine;

namespace Extension
{
    public static class VectorExtensions
    {
        /// <summary>
        /// Clamps the vector with angle
        /// </summary>
        /// <param name="from">Base vector that stays</param>
        /// <param name="to">The vector that rotates</param>
        /// <param name="minAngle">Minimum angle</param>
        /// <param name="maxAngle">Maximum angle</param>
        /// <returns></returns>
        public static Vector3 ClampAngle(this Vector3 from, Vector3 to, float minAngle, float maxAngle)
        {
            return ClampAngle(from, to, Vector3.up, minAngle, maxAngle);
        }

        /// <summary>
        /// Clamps the vector with angle
        /// </summary>
        /// <param name="from">Base vector that stays</param>
        /// <param name="to">The vector that rotates</param>
        /// <param name="worldUp">Up vector, used in dot product to calculate negative and positive angles</param>
        /// <param name="minAngle">Minimum angle</param>
        /// <param name="maxAngle">Maximum angle</param>
        /// <returns></returns>
        public static Vector3 ClampAngle(this Vector3 from, Vector3 to, Vector3 worldUp, float minAngle, float maxAngle)
        {
            Vector3 right = Vector3.Cross(from, to).normalized;
            Vector3 up = Vector3.Cross(right, from).normalized;
            float currentAngle = Vector3.SignedAngle(from, to, right) * Mathf.Sign(Vector3.Dot(up, worldUp));

            return Quaternion.AngleAxis(Mathf.Clamp(currentAngle, minAngle, maxAngle), right * Mathf.Sign(currentAngle)) * from;
        }

        /// <summary>
        /// Clamps the vector with angle in relation to axis
        /// </summary>
        /// <param name="vector">The vector that will clamped</param>
        /// <param name="normal">Axis vector, that rotates other vector around</param>
        /// <param name="minAngle">Minimum angle</param>
        /// <param name="maxAngle">Maximum angle</param>
        /// <returns></returns>
        public static Vector3 ClampAngleAxis(Vector3 vector, Vector3 normal, float minAngle, float maxAngle)
        {
            Vector3 right = Vector3.Cross(normal, vector);
            Vector3 forward = Vector3.Cross(right, normal);
            float currentAngle = Vector3.SignedAngle(forward, vector, right);
            float clampedAngle = Mathf.Clamp(currentAngle, minAngle, maxAngle);

            return Quaternion.AngleAxis(clampedAngle - currentAngle, right) * vector;
        }
    }
}