#pragma once

#include "Representations/Infrastructure/JointRequest.h"

class WalkingEngineOutput : public JointRequest
{
public:
    bool isLeavingPossible = true;
};
