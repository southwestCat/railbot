#pragma once

#include "Tools/Math/Angle.h"
#include "Representations/Infrastructure/JointAngles.h"

class HeadMotionEngineOutput
{
public:
    Angle pan = JointAngles::off;     /**< Head pan target angle. */
    Angle tilt = JointAngles::off;    /**< Head tilt target angle. */
    bool reachable = true; /**< Whether the head motion request points on a reachable position. */
    bool moving = false;   /**< Whether the head is currently in motion or not. */
};