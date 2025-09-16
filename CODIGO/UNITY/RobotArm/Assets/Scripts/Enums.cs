using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Enums
{ 
    public enum ControlMode
    {
        FollowControllerIK,
        JoysticksIK,
        JoysticksFK
    }

    public enum DominantHand
    {
        Right,
        Left,
    }

    public enum VisualizationMode
    {
        Stereoscopic,
        Multiview,
    }

    public enum TunnelingEffect
    {
        No,
        Yes,
    }
}
