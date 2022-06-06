#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Blackboard.h"

class WalkingEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);
};

class WalkingEngine : public WalkingEngineBase
{
public:
};
