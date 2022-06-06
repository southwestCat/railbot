#pragma once

#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Tools/Module/Blackboard.h"

class ComplianceControllerBase
{
public:
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);

    USES_REPRESENTATION(NetWrenchEstimation);
};

class ComplianceController : public ComplianceControllerBase
{
public:
    void update(ComplianceJointRequest &o);

private:
    void update();
};