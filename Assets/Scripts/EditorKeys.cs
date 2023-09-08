using UnityEngine;

namespace BIK
{
    public class EditorKeys : MonoBehaviour
    {
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.C))
            {
                if (Cursor.lockState == CursorLockMode.None)
                    Cursor.lockState = CursorLockMode.Locked;
                else
                    Cursor.lockState = CursorLockMode.None;
            }
        }
    }
}
