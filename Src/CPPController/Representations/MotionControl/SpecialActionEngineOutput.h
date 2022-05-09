#pragma once

#include "Representations/Infrastructure/JointRequest.h"

class SpecialActionEngineOutput : public JointRequest
{
public:
    bool isLeavingPossible = true;
};
