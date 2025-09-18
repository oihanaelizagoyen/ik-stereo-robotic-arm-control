using System;
using System.Linq;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using static Enums;

public class RobotArmController : MonoBehaviour, ITunnelingVignetteProvider
{
    // PUBLIC
    public Enums.ControlMode ControlMode
    {
        get => _controlMode;
        set
        {
            if (_controlMode != value)
            {
                _controlMode = value;

                // Send to neutral position when changing control mode
                MoveRobotArmToNeutralPosition();
            }
        }
    }
    public Enums.DominantHand DominantHand;
    public bool TunnelingEnabled;
    // Vignette parameters for tunneling effect
    public VignetteParameters vignetteParameters { get; private set; }

    // PRIVATE
    // Control mode
    private Enums.ControlMode _controlMode;

    // Input Devices Manager
    private InputDevicesManager _inputDevicesManager;

    // TCP Client Handler
    private TCPClientHandler _tcpClientHandler;

    // TCP reconnection
    private bool _isConnecting = false;
    private float _reconnectionTimer = 0f;
    private const float ReconnectionInterval = 5f; // seconds

    // Sending messages
    private ConcurrentQueue<RobotArmState> _robotArmStateQueue = new ConcurrentQueue<RobotArmState>();
    private const int MaxQueueSize = 100;
    private CancellationTokenSource _cancellationTokenSource = new CancellationTokenSource();
    private Task _robotArmStateSendingTask;
    private float _sendTimer = 0f;
    private const float SendInterval = 1; // 1s
    private bool _timeControlEnabled = true; // ACTIVATE/DEACTIVATE TIME CONTROL (LOG)
    private int _messageCounter = 0;

    // Tunneling controller
    private TunnelingVignetteController _tunnelingVignetteController;

    // Position -> Y and Z axes are swapped to match the robot arm coordinate system
    private Vector3? _controllerPosition;
    private Vector3 _robotArmPosition;
    private Vector3 _neutralPosition = new Vector3(0.0039f, 0.4928f, 0.0118f);
    private Vector3 _lastSentPosition;
    private const float PositionPositiveLimitX = 0.36f;
    private const float PositionNegativeLimitX = -0.36f;
    private const float PositionPositiveLimitY = 0.5f;
    private const float PositionNegativeLimitY = 0.0f;
    private const float PositionPositiveLimitZ = 0.38f;
    private const float PositionNegativeLimitZ = -0.34f;

    // Rotation
    private Quaternion? _controllerRotation;
    private Quaternion _robotArmRotation;
    private Quaternion _neutralRotation = Quaternion.identity;
    private Quaternion _lastSentRotation;

    // Trigger
    private bool _robotArmTriggerPressed;
    private bool _lastSentTriggerPressed;

    // Thresholds
    private const float PositionThreshold = 0.001f; //meters
    private const float RotationThreshold = 1f; //degrees

    // Scale factor for sensitivity (Joysticks)
    private const float PositionScaleFactor = 0.005f; // 1mm

    // Angles -> Forward Kinematics
    private int[] _robotArmAngles;
    private int[] _neutralAngles = new int[5] { 90, 75, 90, 90, 90};
    private int[] _lastSentAngles;
    private const int BaseAnglePositiveLimit = 180;
    private const int BaseAngleNegativeLimit = 0;
    private const int ForeArmAnglePositiveLimit = 150;
    private const int ForeArmAngleNegativeLimit = 0;
    private const int ArmAnglePositiveLimit = 180;
    private const int ArmAngleNegativeLimit = 0;
    private const int LowerWristAnglePositiveLimit = 180;
    private const int LowerWristAngleNegativeLimit = 0;
    private const int UpperWristAnglePositiveLimit = 180;
    private const int UpperWristAngleNegativeLimit = 0;


    // Awake is called the first time the script instance is loaded
    void Awake()
    {
        // Find InputDevicesManager in the scene
        _inputDevicesManager = FindObjectOfType<InputDevicesManager>();
        if (_inputDevicesManager == null)
        {
            Debug.Log("InputDevicesManager not found in the scene");
        }

        // Open TCP connection as client
        _tcpClientHandler = new TCPClientHandler("127.0.0.1", 12345);
        TryConnectAsync();

        // Start the task for sending robot arm state messages
        _cancellationTokenSource = new CancellationTokenSource();
        _robotArmStateSendingTask = Task.Run(() => SendQueuedRobotArmStates(_cancellationTokenSource.Token));

        // Find TunnelingVignetteController in the scene
        _tunnelingVignetteController = FindObjectOfType<TunnelingVignetteController>();
        if (_tunnelingVignetteController == null)
        {
            Debug.Log("TunnelingVignetteController not found in the scene");
        }
        else
        {
            // Get default vignette parameters from the tunneling controller
            vignetteParameters = _tunnelingVignetteController.defaultParameters;
        }

        // Initialize public variables
        ControlMode = Enums.ControlMode.FollowControllerIK;
        DominantHand = Enums.DominantHand.Right;
        TunnelingEnabled = false;

        // Initialize position
        _controllerPosition = null;
        _robotArmPosition = Vector3.zero;
        _lastSentPosition = _neutralPosition;

        // Initialize rotation
        _controllerRotation = null;
        _robotArmRotation = Quaternion.identity;
        _lastSentRotation = _neutralRotation;

        // Initialize trigger
        _robotArmTriggerPressed = false;
        _lastSentTriggerPressed = false;

        // Initialize angles
        _robotArmAngles = (int[])_neutralAngles.Clone();
        _lastSentAngles = (int[])_neutralAngles.Clone();
    }

    // Start is called before the first frame update
    void Start()
    {
        MoveRobotArmToNeutralPosition();
    }

    // Update is called once per frame
    void Update()
    {
        if (_inputDevicesManager == null)
        {
            Debug.Log("InputDevicesManager is null");
            return;
        }
        if(_tcpClientHandler == null)
        {
            Debug.Log("TCPClientHandler is null");
            return;
        }
        if (!_tcpClientHandler.IsConnected())
        {
            Debug.Log("TCPClientHandler is not connected");
            Debug.Log("Retrying to connect to TCP server...");
            _reconnectionTimer += Time.deltaTime;
            if (!_isConnecting && _reconnectionTimer >= ReconnectionInterval)
            {
                _reconnectionTimer = 0f;
                TryConnectAsync();
            }
            return;
        }

        bool isTunnelingNecessary = false;

        if (ControlMode == Enums.ControlMode.FollowControllerIK)
        {
            InputDevice controller = ObtainMainController();
            if (!controller.isValid)
            {
                Debug.Log("Follow Controller Inverse Kinematics Mode - Controller is not valid");
                return;
            }
            if (controller.TryGetFeatureValue(CommonUsages.gripButton, out bool gripping) && gripping)
            {
                UpdateRobotArmPositionFromControllerPosition(controller);
                UpdateRobotArmRotationFromControllerRotation(controller);
            }
            else 
            {
                ResetControllerPositionAndRotation();
            }
            if (controller.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggering))
            {
                _robotArmTriggerPressed = triggering;
            }
        }
        else if (ControlMode == Enums.ControlMode.JoysticksIK)
        {
            float xDisplacement = 0, yDisplacement = 0, zDisplacement = 0;

            InputDevice mainController = ObtainMainController();
            InputDevice secondaryController = ObtainSecondaryController();

            if (!mainController.isValid) 
            { 
                Debug.Log("Joysticks Inverse Kinematics Mode - Main controller is not valid");
            }
            else
            {
                xDisplacement = ExtractJoystickValue(mainController, "X") * PositionScaleFactor;
                zDisplacement = ExtractJoystickValue(mainController, "Y") * PositionScaleFactor; // Y and Z are swapped to match the robot arm coordinate system
                if (mainController.TryGetFeatureValue(CommonUsages.gripButton, out bool gripping) && gripping)
                {
                    UpdateRobotArmRotationFromControllerRotation(mainController);
                }
                else
                {
                    ResetControllerPositionAndRotation(resetPosition: false);
                }
                if (mainController.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggering))
                {
                    _robotArmTriggerPressed = triggering;
                }
            }
            if (!secondaryController.isValid)
            {
                Debug.Log("Joysticks Inverse Kinematics Mode - Secondary controller is not valid");
            }
            else
            {
                yDisplacement = ExtractJoystickValue(secondaryController, "Y") * PositionScaleFactor;
            }
            UpdateRobotArmPositionFromAxisDisplacement(xDisplacement, yDisplacement, zDisplacement);
        }
        else if(ControlMode == Enums.ControlMode.JoysticksFK)
        {
            int baseDisplacement = 0, foreArmDisplacement = 0, armDisplacement = 0, lowerWristDisplacement = 0, upperWristDisplacement = 0;

            InputDevice mainController = ObtainMainController();
            InputDevice secondaryController = ObtainSecondaryController();

            if(!mainController.isValid)
            {
                Debug.Log("Joysticks Forward Kinematics Mode - Main controller is not valid");
            }
            else
            {
                float xValue= ExtractJoystickValue(mainController, "X");
                float yValue = ExtractJoystickValue(mainController, "Y");
                if (Math.Abs(xValue) > Math.Abs(yValue))
                {
                    lowerWristDisplacement = xValue > 0 ? 1 : -1;
                }
                else if (Math.Abs(yValue) > Math.Abs(xValue))
                {
                    upperWristDisplacement = yValue > 0 ? 1 : -1;
                }
                if (mainController.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggering))
                {
                    _robotArmTriggerPressed = triggering;
                }

            }
            if (!secondaryController.isValid)
            {
                Debug.Log("Joysticks Forward Kinematics Mode - Secondary controller is not valid");
            }
            else
            {
                float xValue = ExtractJoystickValue(secondaryController, "X");
                float yValue = ExtractJoystickValue(secondaryController, "Y");
                bool gripping = false;
                secondaryController.TryGetFeatureValue(CommonUsages.gripButton, out gripping);
                if (Math.Abs(xValue) > Math.Abs(yValue))
                {
                    baseDisplacement = xValue > 0 ? 1 : -1;
                }
                else if (Math.Abs(yValue) > Math.Abs(xValue))
                {
                    if (gripping)
                        armDisplacement = yValue > 0 ? 1 : -1;
                    else
                        foreArmDisplacement = yValue > 0 ? 1 : -1;
                }
            }

            int[] anglesDisplacement = new int[5] { baseDisplacement, foreArmDisplacement, armDisplacement, lowerWristDisplacement, upperWristDisplacement };
            for (int i = 0; i < _robotArmAngles.Length; i++)
            {
                _robotArmAngles[i] += anglesDisplacement[i];
            }
            AdjustRobotArmAnglesToLimit();
        }
        else
        {
            Debug.Log("Unknown Control Mode");
            return;
        }

        _sendTimer += Time.deltaTime;
        bool isInverseKinematicsActive = ControlMode.ToString().Contains("IK");
        bool haschanged = isInverseKinematicsActive 
                            ? CheckPositionRotationChange() || (_robotArmTriggerPressed != _lastSentTriggerPressed)
                            : CheckAnglesChange() || (_robotArmTriggerPressed != _lastSentTriggerPressed);

        if (_sendTimer >= SendInterval && haschanged)
        {
            if (EnqueueRobotArmState())
            {
                _sendTimer = 0f;
                _lastSentTriggerPressed = _robotArmTriggerPressed;

                isTunnelingNecessary = isInverseKinematicsActive 
                                    ? CheckPositionRotationChange() 
                                    : CheckAnglesChange();

                if (isInverseKinematicsActive)
                {
                    _lastSentPosition = _robotArmPosition;
                    _lastSentRotation = _robotArmRotation;
                }
                else
                {
                    _lastSentAngles = (int[])_robotArmAngles.Clone();
                }
            }
        }

        ManageTunnelingVignette(isTunnelingNecessary);
    }

    // OnDestroy is called when the MonoBehaviour will be destroyed
    void OnDestroy()
    {
        if (_cancellationTokenSource != null)
            _cancellationTokenSource.Cancel();
        if (_robotArmStateSendingTask != null)
            _robotArmStateSendingTask.Wait();
        if (_tcpClientHandler != null)
            _tcpClientHandler.Disconnect();
    }

    // Try to connect to the TCP server asynchronously
    private async void TryConnectAsync()
    {
        _isConnecting = true;
        await Task.Run(() => _tcpClientHandler.Connect());
        _isConnecting = false;
    }

    // Move the robot arm to a neutral position
    public void MoveRobotArmToNeutralPosition()
    {
        Debug.Log("Moving robot arm to neutral position");

        bool isInverseKinematicsActive = ControlMode.ToString().Contains("IK");

        if (isInverseKinematicsActive)
        {
            _robotArmPosition = _neutralPosition;
            _robotArmRotation = _neutralRotation;
        }
        else
        {
            _robotArmAngles = (int[])_neutralAngles.Clone();
        }

        if (EnqueueRobotArmState())
        {
            _sendTimer = 0f;
            _lastSentTriggerPressed = _robotArmTriggerPressed;

            bool isNecessary = isInverseKinematicsActive 
                                ? CheckPositionRotationChange() 
                                : CheckAnglesChange();

            ManageTunnelingVignette(isNecessary);

            if (isInverseKinematicsActive)
            {
                _lastSentPosition = _robotArmPosition;
                _lastSentRotation = _robotArmRotation;
            }
            else
            {
                _lastSentAngles = (int[])_robotArmAngles.Clone();
            }
        }
    }

    // Check if position or rotation have changed beyond the defined thresholds
    private bool CheckPositionRotationChange()
    {
        bool positionChanged = Vector3.Distance(_robotArmPosition, _lastSentPosition) >= PositionThreshold;
        bool rotationChanged = Quaternion.Angle(_robotArmRotation, _lastSentRotation) >= RotationThreshold;
        return positionChanged || rotationChanged;
    }

    // Check if angles have changed beyond the defined threshold
    private bool CheckAnglesChange()
    {
        bool anglesChanged = _robotArmAngles
                                .Zip(_lastSentAngles, (robotArmAngle, lastSentAngle) => Math.Abs(robotArmAngle - lastSentAngle))
                                .Any(difference => difference > RotationThreshold);
        return anglesChanged;
    }

    // Enqueue the current robot arm state to be sent via TCP
    private bool EnqueueRobotArmState()
    {
        try
        {
            RobotArmState robotArmState = _timeControlEnabled ? new RobotArmStateForTimeControl() : new RobotArmState();

            bool isInverseKinematicsActive = ControlMode.ToString().Contains("IK");
            robotArmState.InverseKinematicsActive = isInverseKinematicsActive ? 1 : 0;
            robotArmState.ClampState = _robotArmTriggerPressed ? 1 : 0;

            if (_timeControlEnabled && robotArmState is RobotArmStateForTimeControl robotArmStateForTimeControl)
            {
                robotArmStateForTimeControl.StateID = _messageCounter;
                _messageCounter++;
                robotArmStateForTimeControl.EqueueTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                robotArmStateForTimeControl.SendTime = 0;
            }

            if (isInverseKinematicsActive)
            {
                robotArmState.Position = new float[] { _robotArmPosition.x, _robotArmPosition.z, _robotArmPosition.y };
                robotArmState.Orientation = new float[] { _robotArmRotation.x, _robotArmRotation.z, _robotArmRotation.y, _robotArmRotation.w };
            }
            else
            {
                robotArmState.Angles = (int[])_robotArmAngles.Clone();
            }

            _robotArmStateQueue.Enqueue(robotArmState);
            Debug.Log($"EnqueueRobotArmState - Enqueued robot arm state: {JsonUtility.ToJson(robotArmState)}");

            return true;
        }
        catch (Exception ex)
        {
            Debug.Log($"EnqueueRobotArmState - Error enqueuing robot arm state: {ex.Message}");
            return false;
        }
    }

    // Send queued robot arm states
    private void SendQueuedRobotArmStates(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested)
        {
            if(_robotArmStateQueue.Count > MaxQueueSize)
            {
                Debug.LogWarning("SendQueuedRobotArmStates - Robot arm state queue is full. Dropping oldest messages.");
                while (_robotArmStateQueue.Count > MaxQueueSize*0.8)
                {
                    _robotArmStateQueue.TryDequeue(out _);
                }
            }
            if (!_tcpClientHandler.IsConnected())
            {
                Debug.Log("SendQueuedRobotArmStates - TCP client is not connected.");
                Task.Delay(10);
                continue;
            }
            if (_robotArmStateQueue.TryDequeue(out RobotArmState robotArmState))
            {
                Debug.Log($"SendQueuedRobotArmStates - Denqueued robot arm state");

                string message;

                if (_timeControlEnabled && robotArmState is RobotArmStateForTimeControl robotArmStateForTimeControl)
                {
                    robotArmStateForTimeControl.SendTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    message = JsonUtility.ToJson(robotArmStateForTimeControl);
                }
                else
                {
                    message = JsonUtility.ToJson(robotArmState);
                }
                
                _tcpClientHandler.SendMessage(message + "\n");
            }
            else
            {
                Task.Delay(10);
            }
        }
    }

    // Obtain the main controller based (dominant hand)
    private InputDevice ObtainMainController()
    {
        if (DominantHand == Enums.DominantHand.Right)
        {
            return _inputDevicesManager.RightController;
        }
        else
        {
            return _inputDevicesManager.LeftController;
        }
    }

    // Obtain the secondary controller (non-dominant hand)
    private InputDevice ObtainSecondaryController()
    {
        if (DominantHand == Enums.DominantHand.Right)
        {
            return _inputDevicesManager.LeftController;
        }
        else
        {
            return _inputDevicesManager.RightController;
        }
    }

    // Update robot arm position based on controller position
    private void UpdateRobotArmPositionFromControllerPosition(InputDevice controller)
    {
        if (controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 actualControllerPosition))
        {
            if(_controllerPosition != null)
            {
                Vector3 deltaPosition = actualControllerPosition - _controllerPosition.Value;
                _robotArmPosition += deltaPosition;
                AdjustRobotArmPositionToLimit();
            }

            _controllerPosition = actualControllerPosition;
        }
    }

    // Update robot arm position based on joystick displacement
    private void UpdateRobotArmPositionFromAxisDisplacement(float xDisplacement, float yDisplacement, float zDisplacement)
    {
        _robotArmPosition += new Vector3(xDisplacement, yDisplacement, zDisplacement);
        AdjustRobotArmPositionToLimit();
    }

    // Extract joystick value from the controller (X or Y axis)
    private float ExtractJoystickValue(InputDevice controller, string axis)
    {

        if (controller.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 joystickValues))
        {
            if (axis == "X")
                return joystickValues.x;
            else if (axis == "Y")
                return joystickValues.y;
        }
        return 0;
    }

    // Adjust robot arm position to be within defined limits
    private void AdjustRobotArmPositionToLimit()
    {
        _robotArmPosition.x = Mathf.Clamp(_robotArmPosition.x, PositionNegativeLimitX, PositionPositiveLimitX);
        _robotArmPosition.y = Mathf.Clamp(_robotArmPosition.y, PositionNegativeLimitY, PositionPositiveLimitY);
        _robotArmPosition.z = Mathf.Clamp(_robotArmPosition.z, PositionNegativeLimitZ, PositionPositiveLimitZ);
    }

    // Update robot arm rotation based on controller rotation
    private void UpdateRobotArmRotationFromControllerRotation(InputDevice controller)
    {
        if (controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion actualControllerRotation))
        {
            if(_controllerRotation != null)
            {
                Quaternion deltaRotation = actualControllerRotation * Quaternion.Inverse(_controllerRotation.Value);
                _robotArmRotation = deltaRotation * _robotArmRotation;
            }
            _controllerRotation = actualControllerRotation;
        }
    }

    private void AdjustRobotArmAnglesToLimit()
    {
        _robotArmAngles[0] = Mathf.Clamp(_robotArmAngles[0], BaseAngleNegativeLimit, BaseAnglePositiveLimit);
        _robotArmAngles[1] = Mathf.Clamp(_robotArmAngles[1], ForeArmAngleNegativeLimit, ForeArmAnglePositiveLimit);
        _robotArmAngles[2] = Mathf.Clamp(_robotArmAngles[2], ArmAngleNegativeLimit, ArmAnglePositiveLimit);
        _robotArmAngles[3] = Mathf.Clamp(_robotArmAngles[3], LowerWristAngleNegativeLimit, LowerWristAnglePositiveLimit);
        _robotArmAngles[4] = Mathf.Clamp(_robotArmAngles[4], UpperWristAngleNegativeLimit, UpperWristAnglePositiveLimit);
    }

    // EStablish that the controller position and rotation are no longer valid
    private void ResetControllerPositionAndRotation(bool resetPosition = true, bool resetRotation = true)
    {
        if (resetPosition)
            _controllerPosition = null;
        if (resetRotation)
            _controllerRotation = null;
    }

    // Manage the tunneling vignette effect based on movement
    private void ManageTunnelingVignette(bool isNecessary)
    {
        if (TunnelingEnabled && _tunnelingVignetteController != null)
        {
            if (isNecessary)
            {
                _tunnelingVignetteController.BeginTunnelingVignette(this);
            }
            else
            {
                _tunnelingVignetteController.EndTunnelingVignette(this);
            }
        }
    }
}
