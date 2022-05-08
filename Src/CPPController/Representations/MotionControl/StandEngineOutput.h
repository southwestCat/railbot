#pragma once

#include "Representations/Infrastructure/JointRequest.h"

class StandEngineOuptut : public JointRequest
{
public:
    bool isLeavingPossible = true;
};