#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Modules/Motion/Utils/NetWrenchObserver.h"
#include "Modules/Motion/Utils/Contact.h"
#include "Tools/Module/Blackboard.h"

class LIPMControllerBase
{
public:
    MODIFIES_REPRESENTATION(FloatingBaseEstimation);
};

class LIPMController
{
public:
    void update(StabilizerJointRequest &s);

private:
    void update();
    void run();
    Contact supportContact();

private:
    NetWrenchObserver netWrenchObs_;
    float leftFootRatio_ = 0.5f;
};