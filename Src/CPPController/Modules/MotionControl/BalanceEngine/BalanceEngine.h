#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Configuration//RobotDimensions.h"
#include "Representations/MotionControl/BalanceEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Motion/BalanceTarget.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/Blackboard.h"

class BalanceEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(DCMJointRequest);
    REQUIRES_REPRESENTATION(FootstepJointRequest);
    REQUIRES_REPRESENTATION(ComplianceJointRequest);
    REQUIRES_REPRESENTATION(BalanceActionSelection);
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
    REQUIRES_REPRESENTATION(RobotModel);

    USES_REPRESENTATION(LegMotionSelection);
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(StiffnessSettings);
    USES_REPRESENTATION(FloatingBaseEstimation);

    MODIFIES_REPRESENTATION(BalanceTarget);
};

class BalanceEngine : public BalanceEngineBase
{
public:
    void update(BalanceEngineOutput &o);

private:
    void update();
    bool readyPosture(BalanceEngineOutput &o);

private:
    const unsigned readyPostureTime = 1000;
    const unsigned readyWaitTime = 500;
    const int hipHeight = MotionConfig::hipHeight;

    float initHeight = 0.f;
    unsigned startTime = 0;
    unsigned nowTime = 0;

    JointAngles startJoints_;
};