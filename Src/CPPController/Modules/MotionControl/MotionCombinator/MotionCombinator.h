#pragma once

#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class MotionCombinatorBase
{
public:
    REQUIRES_REPRESENTATION(HeadMotionEngineOutput);
};

class MotionCombinator : public MotionCombinatorBase
{
public:
    void update(JointRequest &jointRequest);

private:
    void update();
};
