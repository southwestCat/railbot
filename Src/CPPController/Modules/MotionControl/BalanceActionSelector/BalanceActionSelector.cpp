#include "BalanceActionSelector.h"
#include "Tools/Module/ModuleManager.h"
#include <iostream>

void BalanceActionSelector::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
}

void BalanceActionSelector::update(BalanceActionSelection &o)
{
    update();

    switch (handleState())
    {
    case BalanceActionSelection::dcm:
        o.targetAction = BalanceActionSelection::dcm;
        break;
    case BalanceActionSelection::compliance:
        o.targetAction = BalanceActionSelection::compliance;
        break;
    case BalanceActionSelection::footstep:
        o.targetAction = BalanceActionSelection::footstep;
        break;
    default:
        o.targetAction = BalanceActionSelection::dcm;
        break;
    }
}

BalanceActionSelection::BalanceAction BalanceActionSelector::handleState()
{
    if (theKeyStates->pressed[KeyStates::headFront])
    {
        action = BalanceActionSelection::compliance;
    }
    if (theKeyStates->pressed[KeyStates::headMiddle])
    {
        //! update FootstepControllerState info
        Vector3f comPosition = theFloatingBaseEstimation->WTB.translation();
        comPosition.x() -= theRobotDimensions->leftAnkleToSoleCenter.x(); //< Convert coordinates to Contact surface frame.
        theFootstepControllerState->comPosition = comPosition;
        Vector3f comVelocity = theFloatingBaseEstimation->comVelocity;
        theFootstepControllerState->comVelocity = comVelocity;
        theFootstepControllerState->comAcceleration = {0.f, 0.f, 0.f};
        theFootstepControllerState->footSpread = theRobotDimensions->yHipOffset;

        //! Caculate Capture Point

        //! Fuzzy PID
        theFootstepControllerState->stepLength = 200.f;
        theFootstepControllerState->nSteps = 1;
        theFootstepControllerState->leftSwingFirst = true;

        action = BalanceActionSelection::footstep;
    }
    if (theKeyStates->pressed[KeyStates::headRear])
    {
        action = BalanceActionSelection::dcm;
    }
    return action;
}
