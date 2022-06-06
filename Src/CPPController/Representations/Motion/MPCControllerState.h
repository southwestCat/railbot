#pragma once

#include "Tools/Math/Eigen.h"

class MPCControllerState
{
public:
    float stepLength;
    float footSpread;
    float nSteps;
    bool leftSwingFirst;
    
    Vector3f comPosition;
    Vector3f comVelocity;
    Vector3f comAcceleration;
};