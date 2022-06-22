#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Infrastructure/JointRequest.h"

class BalanceTarget
{
public:
    //! Update information of Request
    Pose3f soleLeftRequest;
    Pose3f soleRightRequest;
    JointRequest lastJointRequest;

    //! Update information of DCMController
    bool updatePendulum = false;
};