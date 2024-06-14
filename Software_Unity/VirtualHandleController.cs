using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VirtualHandheldController : MonoBehaviour
{
    // Reference to the SerialTools object for serial communication
    public SerialTools serialTools;

    // Properties to store the velocities of the contact point and handheld grip point
    public Vector3 ContactPointVelocity { get; private set; }
    public Vector3 GripPointVelocity { get; private set; }

    [Header("Virtual Handheld Points")]
    // Transform references for the virtual contact point and handheld grip point
    public Transform VirtualContactPoint; // Virtual contact point at the front end of the handheld, where it collides with objects
    public Transform VirtualGripPoint; // Virtual grip point where the user holds the handheld

    // Variables to store the last positions of the contact and grip points
    private Vector3 lastContactPosition;
    private Vector3 lastGripPosition;

    // Public variables to store waving speeds
    public float wavingSpeed;
    public float gripSpeed;

    // Method to quit the game
    public void QuitGame()
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#endif
        Application.Quit();
    }

    // Start is called before the first frame update
    protected virtual void Start()
    {
        // Check if SerialTools is assigned
        if (serialTools == null)
        {
            Debug.LogError("Serial tools not given! Quitting the game");
            QuitGame();
        }

        // Connect to the device if not already connected
        if (!serialTools.IsConnected) serialTools.Init();
    }

    // FixedUpdate is called once per fixed frame-rate frame
    void FixedUpdate()
    {
        // Check if the virtual handheld points are assigned
        if (VirtualGripPoint != null && VirtualContactPoint != null)
        {
            // Calculate velocities based on the change in position over time
            ContactPointVelocity = (VirtualContactPoint.position - lastContactPosition) / Time.fixedDeltaTime;
            GripPointVelocity = (VirtualGripPoint.position - lastGripPosition) / Time.fixedDeltaTime;

            // Calculate the speeds
            wavingSpeed = Vector3.Magnitude(ContactPointVelocity);
            gripSpeed = Vector3.Magnitude(GripPointVelocity);

            // Store the current positions for the next frame's calculations
            lastContactPosition = VirtualContactPoint.position;
            lastGripPosition = VirtualGripPoint.position;
        }
    }
}
