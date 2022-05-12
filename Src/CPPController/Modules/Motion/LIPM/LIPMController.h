#pragma once

#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class LIPMControllerBase
{
public:
    REQUIRES_REPRESENTATION(NetWrenchEstimation);
};

class LIPMController
{
public:
    void update(StabilizerJointRequest &s);
private:
    void update();
};