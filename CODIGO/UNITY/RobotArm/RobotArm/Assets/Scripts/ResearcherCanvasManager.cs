using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ResearcherCanvasManager : MonoBehaviour
{
    // PUBLIC
    // UI Elements
    public Dropdown ControlModeDropdown;
    public Dropdown DominantHandDropdown;
    public Dropdown VisualizationModeDropdown;
    public Dropdown TunnelingEffectDropdown;
    public Button ApplySetupButton;
    public Button NeutralPositionButton;

    // PRIVATE
    private RobotArmController _robotArmController;
    private WebcamVisualizationManager _webcamVisualizationManager;

    // Awake is called the first time the script instance is loaded
    private void Awake()
    {
        // Get RobotArmController component from the scene
        _robotArmController = FindObjectOfType<RobotArmController>();
        if (_robotArmController == null)
        {
            Debug.LogError("RobotArmController component not found in the scene.");
        }

        // Get WebcamVisualizationManager component from the scene
        _webcamVisualizationManager = FindObjectOfType<WebcamVisualizationManager>();
        if (_webcamVisualizationManager == null)
        {
            Debug.LogError("WebcamVisualizationManager component not found in the scene.");
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        // Initializate dropdown values
        InitDropdownWithEnum(ControlModeDropdown, typeof(Enums.ControlMode));
        InitDropdownWithEnum(DominantHandDropdown, typeof(Enums.DominantHand));
        InitDropdownWithEnum(VisualizationModeDropdown, typeof(Enums.VisualizationMode));
        InitDropdownWithEnum(TunnelingEffectDropdown, typeof(Enums.TunnelingEffect));

        // Add listener to the ApplySetupButton
        ApplySetupButton.onClick.AddListener(OnApplySetupButtonClicked);

        // Add listener to the NeutralPositionButton
        NeutralPositionButton.onClick.AddListener(OnNeutralPositionButtonClicked);
    }

    // Initialize a dropdown with the names of an enum
    private void InitDropdownWithEnum(Dropdown dropdown, System.Type enumType)
    {
        if (!enumType.IsEnum)
        {
            Debug.LogError("Provided type is not an enum.");
            return;
        }
        if (dropdown == null)
        {
            Debug.LogError("Dropdown is null.");
            return;
        }

        dropdown.ClearOptions();
        List<string> options = new List<string>(System.Enum.GetNames(enumType));
        dropdown.AddOptions(options);
        dropdown.value = 0;
        dropdown.RefreshShownValue();
    }

    // Method called when the ApplySetupButton is clicked
    private void OnApplySetupButtonClicked()
    {
        if (_robotArmController == null)
        {
            Debug.LogError("Cannot apply all the setup because RobotArmController component is missing.");
        }
        else
        {
            _robotArmController.ControlMode = (Enums.ControlMode)ControlModeDropdown.value;
            _robotArmController.DominantHand = (Enums.DominantHand)DominantHandDropdown.value;
            Enums.TunnelingEffect tunnelingEffect = (Enums.TunnelingEffect)TunnelingEffectDropdown.value;
            _robotArmController.TunnelingEnabled = tunnelingEffect == Enums.TunnelingEffect.Yes;
        }
        if (_webcamVisualizationManager == null)
        {
            Debug.LogError("Cannot apply all the setup because WebcamVisualizationManager component is missing.");
        }
        else
        {
            _webcamVisualizationManager.VisualizationMode = (Enums.VisualizationMode)VisualizationModeDropdown.value;
        }
    }

    // Method called when the NeutralPositionButton is clicked
    private void OnNeutralPositionButtonClicked()
    {
        if (_robotArmController == null)
        {
            Debug.LogError("Cannot set neutral position because RobotArmController component is missing.");
        }
        else
        {
            _robotArmController.MoveRobotArmToNeutralPosition();
        }
    }
}
