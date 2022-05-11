#pragma once

#include "Representations/Infrastructure/JointRequest.h"

class BalanceEngineOutput : public JointRequest
{
public:
    bool isLeavingPossible = false;
};