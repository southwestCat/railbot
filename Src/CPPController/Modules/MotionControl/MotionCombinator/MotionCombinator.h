#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Tools/Module/Blackboard.h"

class MotionCombinatorBase
{
public:
    REQUIRES_REPRESENTATION(HeadJointRequest);
    REQUIRES_REPRESENTATION(StiffnessSettings);
};

class MotionCombinator : public MotionCombinatorBase
{
public:
    void update(JointRequest &jointRequest);

private:
    void update();
};
