#pragma once

#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Blackboard.h"

class ComplianceControllerBase
{
public:
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(BalanceActionSelection);
    REQUIRES_REPRESENTATION(JointAngles);

    USES_REPRESENTATION(NetWrenchEstimation);
};

class ComplianceController : public ComplianceControllerBase
{
public:
    ComplianceController();
    void update(ComplianceJointRequest &o);

private:
    void update();

private:
    float gyroThreshold;
    bool keepJoints;
};