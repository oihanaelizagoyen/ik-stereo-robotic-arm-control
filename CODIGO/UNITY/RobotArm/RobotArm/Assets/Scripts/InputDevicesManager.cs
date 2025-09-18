using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class InputDevicesManager : MonoBehaviour
{
    public InputDevice RightController;
    public InputDevice LeftController;
    public InputDevice Headset;

    // Called always the object becomes enabled and active
    private void OnEnable()
    {
        // Initialize all inputs devices at startup
        TryInitializeAllInputDevices();

        // Add listeners for device conenction and disconnection events
        InputDevices.deviceConnected += OnDeviceConnected;
        InputDevices.deviceDisconnected += OnDeviceDisconnected;
    }

    // Called always the object becomes disabled or inactive
    private void OnDisable()
    {
        // Remove listeners for device connection and disconnection events
        InputDevices.deviceConnected -= OnDeviceConnected;
        InputDevices.deviceDisconnected -= OnDeviceDisconnected;

        // Clear references to input devices
        RightController = default;
        LeftController = default;
        Headset = default;
    }

    // Try to initialize all input devices at startup
    private void TryInitializeAllInputDevices()
    {
        Debug.Log("Initializing input devices...");

        TryInitializeInputDevice(InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller, ref RightController);
        TryInitializeInputDevice(InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller, ref LeftController);
        TryInitializeInputDevice(InputDeviceCharacteristics.HeadMounted, ref Headset);
    }

    // Try to initialize an input device with given characteristics
    private void TryInitializeInputDevice(InputDeviceCharacteristics characteristics, ref InputDevice device)
    {
        Debug.Log($"Trying to initialize device with characteristics: {characteristics}");

        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(characteristics, devices);
        
        if (devices.Count > 0)
        {
            device = devices[0];
            Debug.Log($"Initialized device: {device.name} with characteristics: {characteristics}");
        }
        else
        {
            Debug.LogWarning($"No device found with characteristics: {characteristics}");
        }
    }

    // Callback for device connection event
    private void OnDeviceConnected(InputDevice device)
    {
        Debug.Log($"Device connected: {device.name} with characteristics: {device.characteristics}");

        if (device.characteristics.HasFlag(InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller))
        {
            RightController = device;
        }
        else if (device.characteristics.HasFlag(InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller))
        {
            LeftController = device;
        }
        else if (device.characteristics.HasFlag(InputDeviceCharacteristics.HeadMounted))
        {
            Headset = device;
        }
    }

    // Callback for device disconnection event
    private void OnDeviceDisconnected(InputDevice device)
    {
        Debug.Log($"Device disconnected: {device.name} with characteristics: {device.characteristics}");

        if (device == RightController)
        {
            RightController = default;
        }
        else if (device == LeftController)
        {
            LeftController = default;
        }
        else if (device == Headset)
        {
            Headset = default;
        }
    }
}
