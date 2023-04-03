//Holds all global variables of PIDController.cs
//PIDController can calculate multiple object's PID with this implementation
[System.Serializable]
public class PIDStore<T>
{
    public T integrationStored;

    public T errorLast;
    public T valueLast;

    public bool derivativeInitialized;

    public void ResetPID()
    {
        derivativeInitialized = false;
    }
}
