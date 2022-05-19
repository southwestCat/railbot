#pragma once

#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Tools/Module/Blackboard.h"

class SpecialActionEngineBase
{
public:
    USES_REPRESENTATION(SpecialActionRequest);
};

class SpecialActionEngine : public SpecialActionEngineBase
{
public:
    void update(SpecialActionEngineOutput &s);
};
