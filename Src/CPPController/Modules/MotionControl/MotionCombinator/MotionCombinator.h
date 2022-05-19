#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/SitDownEngineOutput.h"
#include "Representations/MotionControl/BalanceEngineOutput.h"
#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Tools/Module/Blackboard.h"

class MotionCombinatorBase
{
public:
    REQUIRES_REPRESENTATION(HeadJointRequest);
    REQUIRES_REPRESENTATION(LegJointRequest);
    REQUIRES_REPRESENTATION(LegMotionSelection);
    REQUIRES_REPRESENTATION(StandEngineOuptut);
    REQUIRES_REPRESENTATION(SitDownEngineOutput);
    REQUIRES_REPRESENTATION(SpecialActionEngineOutput);

    USES_REPRESENTATION(StiffnessSettings);
    USES_REPRESENTATION(JointLimits);
    USES_REPRESENTATION(MotionRequest);
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
