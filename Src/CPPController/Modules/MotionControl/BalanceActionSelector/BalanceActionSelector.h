#pragma once

#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Motion/FootstepControllerState.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Motion/BalanceTarget.h"
#include "Modules/Motion/Footstep/FuzzyPID.h"
#include "Tools/Module/Blackboard.h"
#include "SelectorConfig.h"

#include "CABSL/Cabsl.h"

class BalanceActionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);

    USES_REPRESENTATION(KeyStates);
    USES_REPRESENTATION(FloatingBaseEstimation);
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(BalanceTarget);

    MODIFIES_REPRESENTATION(FootstepControllerState);
};

class BalanceActionSelector : public BalanceActionSelectorBase, public Cabsl<BalanceActionSelector>
{
public:
    BalanceActionSelector();
    void update(BalanceActionSelection &b);

private:
    void update();
    BalanceActionSelection::BalanceAction handleState();
    BalanceActionSelection::BalanceAction autoHandle();
    BalanceActionSelection::BalanceAction mannualHandle();

    bool comInInitialPosition();
    bool footstepsAction();

private:
    BalanceActionSelection::BalanceAction action = BalanceActionSelection::compliance;
    BalanceActionSelection::BalanceAction lastAction;
    FuzzyPID fuzzyPID;

    float footstepInitialXd;

    ActivationGraph activationGraph;

    unsigned stayInCoMInitialState = 0;
    unsigned comVelocityMaxCounter = 0;
#include "ActionSelectorOptions.h"
};
