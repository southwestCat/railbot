#pragma once

#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Module/Blackboard.h"

class SpecialActionEngineBase
{
public:
    USES_REPRESENTATION(MotionRequest);
};

class SpecialActionEngine : public SpecialActionEngineBase
{
public:
    void update(SpecialActionEngineOutput &s);
};
