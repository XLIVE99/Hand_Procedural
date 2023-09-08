using UnityEngine;

namespace BIK
{
    public static class QuaternionExtensions
    {
        public static Quaternion ScalarMultiply(this Quaternion q, float scalar)
        {
            //Same with javascript
            q.x *= scalar;
            q.y *= scalar;
            q.z *= scalar;
            q.w *= scalar;
            return q;
        }
    }
}
