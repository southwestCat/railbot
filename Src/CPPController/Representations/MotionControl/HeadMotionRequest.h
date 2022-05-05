#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Infrastructure/JointAngles.h"

class HeadMotionRequest
{
public:
    Angle pan = JointAngles::off;   /**< Head pan target angle in radians. */
    Angle tilt = JointAngles::off;  /**< Head tilt target angle in radians. */
    Angle speed = 1_deg; /**< Maximum joint speed to reach target angles in radians/s. */
};
