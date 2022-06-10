#pragma once

#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Motion/FootstepControllerState.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Modules/Motion/Footstep/FuzzyPID.h"
#include "Tools/Module/Blackboard.h"

class BalanceActionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);

    USES_REPRESENTATION(KeyStates);
    USES_REPRESENTATION(FloatingBaseEstimation);
    USES_REPRESENTATION(RobotDimensions);

    MODIFIES_REPRESENTATION(FootstepControllerState);
};

class BalanceActionSelector : public BalanceActionSelectorBase
{
public:
    void update(BalanceActionSelection &b);

private:
    void update();
    BalanceActionSelection::BalanceAction handleState();

private:
    BalanceActionSelection::BalanceAction action = BalanceActionSelection::compliance;
    FuzzyPID fuzzyPID;
};
