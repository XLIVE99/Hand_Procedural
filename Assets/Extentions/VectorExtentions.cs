using UnityEngine;

public static class VectorExtentions
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
        Vector3 cross = Vector3.Cross(from, to).normalized;
        Vector3 up = Vector3.Cross(cross, from).normalized;
        float currentAngle = Vector3.SignedAngle(from, to, cross) * Mathf.Sign(Vector3.Dot(up, worldUp));

        return Quaternion.AngleAxis(Mathf.Clamp(currentAngle, minAngle, maxAngle), cross * Mathf.Sign(currentAngle)) * from;
    }
}
