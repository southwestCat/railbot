#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Tools/Module/Blackboard.h"

class HeadMotionCombinatorBase
{
public:
    REQUIRES_REPRESENTATION(HeadMotionEngineOutput);
};

class HeadMotionCombinator : public HeadMotionCombinatorBase
{
public:
    void update(HeadJointRequest &jointRequest);

private:
    void update();
};