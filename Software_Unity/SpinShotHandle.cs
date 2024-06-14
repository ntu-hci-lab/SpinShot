using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using System.Threading;
using System.Drawing.Text;

[RequireComponent(typeof(SerialTools))]
public class SpinShotHandle : VirtualHandheldController
{
    /* inspector */
    [Header("Physical Handle")]
    public string releaseRPM = "50";
    public const int MinimumImpactRpm = 150;
    public float swingImpactDelay;

    [Header("ADT-based Design Parameters")]
    [Tooltip("A defined swing speed. At this swing speed, the user is expected not to perceive the device's maximum output acceleration of the flywheel within this application.")]
    public float MaxPerceivableGripSpeed = 5;  // The maximum grip speed that can be perceived by the user

    public enum AccelerationMode { constant, linear, square }  // Different modes of acceleration
    public enum HandednessTypes { leftHanded, rightHanded }  // Types of handedness

    [Header("Mode")]
    public AccelerationMode Mode;
    public HandednessTypes Handedness;

    [Serializable]
    public struct DeviceState
    {
        public int Rpm;
        public int Angle;
        public int GivenCurrent;
        public enum RunningStatus { FREE = 0, STABLE = 1, ACCELERATE = 2, WAIT = 3, CALIBRATE = 4, IMPACT = 5, IMPACT_AFTER = 6, TEST = 7, NOT_CALIBRATED = 8, ERR = 9 }
        public RunningStatus Status;
    }
    public DeviceState deviceState;

    /* private */
    protected int handleTargetRpm = 0;
    public int TargetRpm { get { return handleTargetRpm; } }
    protected int acceptableCurrent;
    private Thread serialReading;

    protected override void Start()
    {
        base.Start();
        serialReading = new Thread(ReadDeviceStatus);
        serialReading.Start();
    }

    private void OnApplicationQuit()
    {
        serialTools.wantToClear = true;
        serialTools.SerialWrite("free ", "end ");
        serialReading.Abort();
    }

    public void Free()
    {
        serialTools.SerialWriteLines(new string[] { "free " });
    }

    public void Wait()
    {
        serialTools.SerialWriteLines(new string[] { "wait " });
    }

    public void Accelerate(int targetRpm)
    {
        acceptableCurrent = GetAcceptableCurrent();
        Accelerate(targetRpm, acceptableCurrent);
    }

    public virtual void Accelerate(int targetRpm, int useCurrent)
    {
        serialTools.SerialWriteLines(new string[] { "acc " + targetRpm.ToString() + " " + useCurrent.ToString() + " " });
        handleTargetRpm = targetRpm;
    }

    public void ImpactAndFree()
    {
        if (Math.Abs(handleTargetRpm) < MinimumImpactRpm)
        {
            Debug.LogError("RPM is too low to impact!");
            serialTools.wantToClear = true;
        }
        else if (handleTargetRpm > 0)
        {
            // The flywheel will reverse a little bit to make sure the stopper is able to retract
            serialTools.SerialWriteLines(new string[] { "impact acc -" + releaseRPM + " 900 wait acc 0 500 " });
        }
        else if (handleTargetRpm < 0)
        {
            serialTools.SerialWriteLines(new string[] { "impact acc " + releaseRPM + " 900 wait acc 0 500 " });
        }
    }

    public void ImpactAndReverse(int reverseRpm, int reverseCurrent)
    {
        if (Math.Abs(handleTargetRpm) < MinimumImpactRpm)
        {
            Debug.LogError("RPM is too low to impact!");
            serialTools.wantToClear = true;
        }
        else if (handleTargetRpm > 0) serialTools.SerialWriteLines(new string[] { "impact acc " + (-Math.Abs(reverseRpm)).ToString() + " " + reverseCurrent.ToString() + " " + "wait acc 0 300 " });
        else if (handleTargetRpm < 0) serialTools.SerialWriteLines(new string[] { "impact acc " + Math.Abs(reverseRpm).ToString() + " " + reverseCurrent.ToString() + " " + "wait acc 0 300 " });
    }

    public void TestDevice()
    {
        serialTools.SerialWriteLines(new string[] { "acc 100 1000 wait free " });
    }

    public void ToggleHandedness(bool isLeftHanded)
    {
        if (isLeftHanded)
        {
            Handedness = HandednessTypes.leftHanded;
        }
        else
        {
            Handedness = HandednessTypes.rightHanded;
        }
    }

    private void ReadDeviceStatus()
    {
        string message = "";
        string[] splitMessages = new string[5];
        bool[] parseState = new bool[5];
        int runningStatusInt = 0;
        while (true)
        {
            Thread.Sleep(1);
            message = serialTools.SerialReadMessage();
            if (message.Length == 14)
            {
                splitMessages[0] = message.Substring(0, 4);
                splitMessages[1] = message.Substring(4, 3);
                splitMessages[2] = message.Substring(7, 5);
                splitMessages[3] = message.Substring(12, 1);
                splitMessages[4] = message.Substring(13, 1);
                parseState[0] = int.TryParse(splitMessages[0], out deviceState.Rpm);
                parseState[1] = int.TryParse(splitMessages[1], out deviceState.Angle);
                parseState[2] = int.TryParse(splitMessages[2], out deviceState.GivenCurrent);
                parseState[3] = int.TryParse(splitMessages[3], out runningStatusInt);
                deviceState.Status = (DeviceState.RunningStatus)runningStatusInt;
            }
            else if (message.Length == 7)
            {
                // Check for success or failure messages
                if (message == "succ001")
                {
                    lock (serialTools.writeLock)
                    {
                        serialTools.cmdSuccOrFail = true;
                        serialTools.isACKed = true;
                    }
                }
                else if (message == "fail001")
                {
                    lock (serialTools.writeLock)
                    {
                        serialTools.cmdSuccOrFail = false;
                        serialTools.isACKed = true;
                    }
                }
                message = "";
            }
        }
    }

    private int GetAcceptableCurrent()
    {
        float current = 0;
        if (Mode == AccelerationMode.linear)
        {
            current = Mathf.Lerp(900, 10000, gripSpeed / MaxPerceivableGripSpeed); 
        }
        else if (Mode == AccelerationMode.constant)
        {
            current = 900;
        }
        return (int)current;
    }
}

