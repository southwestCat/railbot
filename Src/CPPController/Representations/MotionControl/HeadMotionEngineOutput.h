#pragma once

#include "Tools/Math/Angle.h"

class HeadMotionEngineOutput
{
public:
    Angle pan = 0_deg;     /**< Head pan target angle. */
    Angle tilt = 0_deg;    /**< Head tilt target angle. */
    bool reachable = true; /**< Whether the head motion request points on a reachable position. */
    bool moving = false;   /**< Whether the head is currently in motion or not. */
};