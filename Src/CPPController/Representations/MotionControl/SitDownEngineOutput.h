#pragma once

#include "Representations/Infrastructure/JointRequest.h"

class SitDownEngineOutput : public JointRequest
{
public:
    bool isLeavingPossible = false;
};