#pragma once

#include "Tools/Math/Eigen.h"

class FootstepControllerState
{
public:
    float stepLength;
    float footSpread;
    unsigned nSteps;
    bool leftSwingFirst;
    
    Vector3f comPosition;
    Vector3f comVelocity;
    Vector3f comAcceleration;
};