using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotArmStateForTimeControl : RobotArmState
{
    // Integer for state ID
    public int StateID;
    // Long for enqueue time
    public long EqueueTime;
    // Long for send time
    public long SendTime;
}
