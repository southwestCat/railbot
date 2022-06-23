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
        break;
    }
}

BalanceActionSelection::BalanceAction BalanceActionSelector::handleState()
{
    //! Configure FuzzyPID
    static bool once = true;
    if (once)
    {
        fuzzyPID.setERange(-30.f, 30.f);
        fuzzyPID.setECRange(-10.f, 10.f);
        fuzzyPID.calculate();
        once = false;
    }

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
        Vector3f hipPosition = theFloatingBaseEstimation->WTO.translation();
        hipPosition.x() -= theRobotDimensions->leftAnkleToSoleCenter.x(); //< Convert coordinates to Contact surface frame.
        theFootstepControllerState->hipPosition = hipPosition;
        Vector3f comVelocity = theFloatingBaseEstimation->comVelocity;
        theFootstepControllerState->comVelocity = comVelocity;
        theFootstepControllerState->comAcceleration = {0.f, 0.f, 0.f};
        theFootstepControllerState->footSpread = theRobotDimensions->yHipOffset;

        //! Fuzzy PID
        float x = comPosition.x();
        float xd = comVelocity.x();
        float yd = comVelocity.y();
        float stepLength = fuzzyPID.getU(x, xd);
        
        printf(">\n");
        printf(" com: %3.3f %3.3f %3.3f\n", comPosition.x(), comPosition.y(), comPosition.z());
        printf("comd: %3.3f %3.3f %3.3f\n", comVelocity.x(), comVelocity.y(), comVelocity.z());
        printf("----\n\n");

        //! Set Footstep params
        theFootstepControllerState->stepLength = stepLength;
        theFootstepControllerState->nSteps = 1;
        theFootstepControllerState->leftSwingFirst = (yd >= 0.f ? true : false);

        action = BalanceActionSelection::footstep;
    }
    if (theKeyStates->pressed[KeyStates::headRear])
    {
        action = BalanceActionSelection::dcm;
    }
    return action;
}
