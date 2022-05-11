#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Configuration//RobotDimensions.h"
#include "Representations/MotionControl/BalanceEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Blackboard.h"

class BalanceEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(InertialData);

    USES_REPRESENTATION(LegMotionSelection);
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(StiffnessSettings);

    unsigned startTime = 0;
    unsigned nowTime = 0;
};

class BalanceEngine : public BalanceEngineBase
{
public:
    void update(BalanceEngineOutput &o);

private:
    void update();
};