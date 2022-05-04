#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"

class HeadMotionRequest
{
public:
    Angle pan = 0_deg;   /**< Head pan target angle in radians. */
    Angle tilt = 0_deg;  /**< Head tilt target angle in radians. */
    Angle speed = 1_deg; /**< Maximum joint speed to reach target angles in radians/s. */
};
