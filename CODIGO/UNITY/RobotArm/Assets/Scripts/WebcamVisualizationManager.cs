using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

public class WebcamVisualizationManager : MonoBehaviour
{
    // PUBLIC
    public Enums.VisualizationMode VisualizationMode
    {
        get => _visualizationMode;
        set
        {
            if (_visualizationMode != value)
            {
                _visualizationMode = value;

                // Update the material property to reflect the new visualization mode
                Renderer renderer = GetComponent<Renderer>();
                Material material = renderer.material;
                material.SetInt("_Stereoscopic", _visualizationMode == Enums.VisualizationMode.Stereoscopic ? 1 : 0);
            }
        }
    }

    // PRIVATE
    private Enums.VisualizationMode _visualizationMode;
    private WebCamTexture _leftUpWebcamTexture;
    private WebCamTexture _rightDownWebcamTexture;
    private float reconnectTimer = 0f;
    private const float ReconnectDelay = 2f;
    private float _webcamAspectRatio;
    private float _doubleWebcamAspectRatio;
    private const int DesiredWebcamHeight = 1080;
    private const int DesiredWebcamWidth = 1920;
    private const int DesiredWebcamFPS = 60;
    private const float MultiviewScaleFactor = 0.65f;

    // Awake is called the first time the script instance is loaded
    private void Awake()
    {
        // Initialize visualization mode
        VisualizationMode = Enums.VisualizationMode.Stereoscopic;

        // Request permission to use the webcam
        #if UNIT_WEBPLAYER || UNITY_FLASH
            yield Application.RequestUserAuthorization(UserAuthorization.WebCam);
        #endif

        StartWebcams();
    }

    // Update is called once per frame
    void Update()
    {
        UpdateObjectScale();

        if (_leftUpWebcamTexture == null || !_leftUpWebcamTexture.isPlaying || 
            _rightDownWebcamTexture == null || !_rightDownWebcamTexture.isPlaying)
        {
            reconnectTimer += Time.deltaTime;
            if (reconnectTimer >= ReconnectDelay)
            {
                reconnectTimer = 0f;
                Debug.LogWarning("Webcam lost or not initialized, attempting to reconnect...");
                StartWebcams();
            }
        }
    }

    // Start the webcams and assign their textures to the material
    private void StartWebcams()
    {
        // Obtain the list of available webcam devices
        WebCamDevice[] devices = WebCamTexture.devices;

        // Check if there are at least two devices
        if (devices.Length < 2)
        {
            Debug.Log("StartWebcams - Not enough webcam devices found");
            return;
        }

        // Stop and release existing webcam texture
        if (_leftUpWebcamTexture != null)
        {
            _leftUpWebcamTexture.Stop();
            _leftUpWebcamTexture = null;
        }

        // Stop and release existing webcam texture
        if (_rightDownWebcamTexture != null)
        {
            _rightDownWebcamTexture.Stop();
            _rightDownWebcamTexture = null;
        }

        // Obtain texture from the first two devices
        _leftUpWebcamTexture = new WebCamTexture(devices[0].name, DesiredWebcamWidth, DesiredWebcamHeight, DesiredWebcamFPS);
        _rightDownWebcamTexture = new WebCamTexture(devices[1].name, DesiredWebcamWidth, DesiredWebcamHeight, DesiredWebcamFPS);

        // Assign the textures to the material
        Renderer renderer = GetComponent<Renderer>();
        Material material = renderer.material;
        material.SetTexture("_LeftUpTex", _leftUpWebcamTexture);
        material.SetTexture("_RightDownTex", _rightDownWebcamTexture);

        // Start the webcam textures
        _leftUpWebcamTexture.Play();
        _rightDownWebcamTexture.Play();

        // Calculate the aspect ratio of the webcam texture
        _webcamAspectRatio = (float)_leftUpWebcamTexture.width / (float)_leftUpWebcamTexture.height;
        _doubleWebcamAspectRatio = _webcamAspectRatio * 0.5f; // Two images, one on top of the other
    }

    // Update the scale of the object based on the main camera's height and width at the object's Z position
    private void UpdateObjectScale()
    {
        if(!ObtainMainCameraHeightAndWidthAtZ(transform.localPosition.z, out float cameraHeight, out float cameraWidth))
        {
            Debug.Log("UpdateObjectScale - Could not obtain camera height and width");
            return;
        }

        Vector3 newScale;
        // Stereoscopic: We want the image to cover all the width, even though it is not complete.
        if (VisualizationMode == Enums.VisualizationMode.Stereoscopic)
        {
            // Scale to match camera width
            newScale = new Vector3(cameraWidth, cameraWidth / _webcamAspectRatio, 1.0f);
        }
        // Multiview: We want the image to be fully visible, even though there will be black bars.
        else if (VisualizationMode == Enums.VisualizationMode.Multiview)
        {
            float cameraAspectRatio = cameraWidth / cameraHeight;

            // At same height, camera width is smaller
            if (cameraAspectRatio < _webcamAspectRatio)
            {
                // Scale to match camera width
                newScale = new Vector3(cameraWidth * MultiviewScaleFactor, (cameraWidth / _doubleWebcamAspectRatio) * MultiviewScaleFactor, 1.0f);
            }
            // At same width, camera height is smaller
            else
            {
                // Scale to match camera height
                newScale = new Vector3((cameraHeight * _doubleWebcamAspectRatio) * MultiviewScaleFactor, cameraHeight * MultiviewScaleFactor, 1.0f);
            }
        }
        else
        {
            Debug.Log("UpdateObjectScale - Unknown visualization mode");
            return;
        }

        transform.localScale = newScale;
    }

    // Obtain the height and width of the main camera at a given Z distance
    private bool ObtainMainCameraHeightAndWidthAtZ(float z, out float height, out float width)
    {
        height = 0;
        width = 0;

        // Obtain the component camera of an Object called "Main Camera"
        Camera camera = GameObject.Find("Main Camera").GetComponent<Camera>();
        if (camera == null)
        {
            Debug.Log("ObtainMainCameraHeightAndWidthAtZ - Main Camera not found");
            return false;
        }

        if (camera.orthographic)
        {
            height = camera.orthographicSize * 2.0f;
        }
        else
        {
            // tan = opposite/adjacent => opposite = tan * z
            height = 2.0f * Mathf.Abs(z) * Mathf.Tan(camera.fieldOfView * 0.5f * Mathf.Deg2Rad);
        }

        width = height * camera.aspect;

        return true;
    }

    [ContextMenu("See Devices")]
    public void seeDevices()
    {
        WebCamDevice[] devices = WebCamTexture.devices;
        foreach (WebCamDevice device in devices)
        {
            Debug.Log("Device found: " + device.name);
        }
    }
}
