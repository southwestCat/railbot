#pragma once

#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class ComplianceControllerBase
{
public:
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
};

class ComplianceController : public ComplianceControllerBase
{
public:
    void update(ComplianceJointRequest &o);

private:
    void update();
};