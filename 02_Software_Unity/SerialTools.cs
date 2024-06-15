using System;
using System.IO.Ports;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor.VersionControl;
using System.Threading;

public class SerialTools : MonoBehaviour
{
    public SerialPort serialStream;
    public List<string> messageQueue = new List<string>();

    [Header("Controller Serial Settings")]
    public string PortName = "COM5";
    public int BaudRate = 115200;

    [Header("Other Settings")]
    public int ReadTimeout = 10;

    public bool IsConnected { get; private set; } = false;
    public bool cmdSuccOrFail;
    public bool isACKed;
    public bool wantToClear = false;

    public object writeLock = new object();
    Thread serialWriting = null;

    public void Init()
    {
        serialStream = new SerialPort(PortName, BaudRate)
        {
            ReadTimeout = ReadTimeout
        };

        try
        {
            serialStream.Open();
            Debug.Log("SerialPort connected");
            IsConnected = true;
            serialWriting = new Thread(MessageWriting);
            serialWriting.Start();
        }
        catch (System.Exception e)
        {
            Debug.LogError("SerialPort connection failed");
            Debug.LogError(e.Message);
        }
    }

    public void SerialWrite(string message)
    {
        if (serialStream.IsOpen)
        {
            try
            {
                serialStream.Write(message);
            }
            catch (System.Exception e)
            {
                Debug.LogError(e.Message);
            }
        }
    }
    public void SerialWrite(string message, string ending)
    {
        if (serialStream.IsOpen)
        {
            try
            {
                serialStream.Write(message + ending);
            }
            catch (System.Exception e)
            {
                Debug.LogError(e.Message);
            }
        }
    }

    public void SerialWriteLines(string[] messages)
    {
        lock (writeLock)
        {
            foreach (string message in messages)
            {
                messageQueue.Add(message);
            }
        }
    }
    private void MessageWriting()
    {
        // need to give the list of messages to sent block by block as a whole, otherwise they will be sent parallelly without being blocked
        // now we always send message at index 0
        isACKed = true;
        cmdSuccOrFail = false;
        while (true)
        {
            if (messageQueue.Count != 0)
            {
                if (isACKed)
                {
                    if (cmdSuccOrFail)
                    {
                        // succ, next message
                        lock (writeLock)
                        {
                            messageQueue.RemoveAt(0);
                            cmdSuccOrFail = false;
                        }
                    }
                    else
                    {
                        //fail, resend current message
                        lock (writeLock)
                        {
                            isACKed = false;
                            SerialWrite(messageQueue[0], "end ");
                        }
                        Thread.Sleep(20);
                    }
                }
                //check if one want to clear Queue
                if (wantToClear)
                {
                    messageQueue.Clear();
                    wantToClear = false;
                    messageQueue.Add("free ");
                }
            }
            else
            {
                //do nothing
                Thread.Sleep(20);
            }
        }
    }

    public string SerialReadLine()
    {
        if (serialStream.IsOpen)
        {
            try
            {
                return serialStream.ReadLine();
            }
            catch (System.Exception e)
            {
                Debug.LogError(e.Message);
            }
        }
        return "";
    }

    public string SerialReadMessage()
    {
        if (serialStream.IsOpen)
        {
            try
            {
                return serialStream.ReadTo("\0");
            }
            catch (System.Exception e)
            {
                Debug.LogError(e.Message);
            }
        }
        return "";
    }

    public void OnApplicationQuit()
    {

        if (serialWriting != null)
        {
            serialWriting.Abort();
        }
        if (serialStream != null)
        {
            if (serialStream.IsOpen)
            {
                serialStream.Close();
                Debug.Log("Close Connection");
            }
        }
    }
}
