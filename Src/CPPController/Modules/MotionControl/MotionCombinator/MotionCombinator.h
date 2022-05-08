#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/MotionControl/MotionInfo.h"
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
    void update(MotionInfo &motionInfo);
private:
    void update();
    MotionInfo motionInfo;
};
