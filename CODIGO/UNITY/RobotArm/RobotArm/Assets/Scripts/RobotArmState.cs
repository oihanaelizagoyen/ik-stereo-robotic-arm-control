using JetBrains.Annotations;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class RobotArmState
{
    // Integer for inverse kinematics active (0 = false, 1 = true)
    public int InverseKinematicsActive;
    // Vector of 5 integers for joint angles (in degrees)
    public int[] Angles;
    // Vector of 3 floats for position (x, z, y)
    public float[] Position;
    // Vector of 4 floats for orientation (x, z, y, w)
    public float[] Orientation;
    // Integer for clamp state (0 = open, 1 = closed)
    public int ClampState;
}
