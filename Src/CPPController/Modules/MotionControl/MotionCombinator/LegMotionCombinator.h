#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Tools/Module/Blackboard.h"

class LegMotionCombinatorBase
{
public:
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(LegMotionSelection);
    REQUIRES_REPRESENTATION(WalkingEngineOutput);
    REQUIRES_REPRESENTATION(StandEngineOuptut);   
    REQUIRES_REPRESENTATION(SpecialActionEngineOutput);

    USES_REPRESENTATION(StiffnessSettings);
};

class LegMotionCombinator : public LegMotionCombinatorBase
{
public:
    void update(LegJointRequest &legJointRequest);

private:
    void update();
    JointAngles lastJointAngles;
};