#include "BalanceActionSelector.h"
#include "Tools/Module/ModuleManager.h"
#include <iostream>

BalanceActionSelector::BalanceActionSelector() : Cabsl<BalanceActionSelector>(&activationGraph)
{
}

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
    // return mannualHandle();
    return autoHandle();
}

BalanceActionSelection::BalanceAction BalanceActionSelector::autoHandle()
{
    //! Configure FuzzyPID
    static bool once = true;
    if (once)
    {
        const float ea = Selector::Ea;
        const float eb = Selector::Eb;
        const float ec = Selector::Ec;
        const float ed = Selector::Ed;
        const float eca = Selector::ECa;
        const float ecb = Selector::ECb;
        const float ecc = Selector::ECc;
        const float ecd = Selector::ECd;

        fuzzyPID.setERange(ea, eb, ec, ed);
        fuzzyPID.setECRange(eca, ecb, ecc, ecd);
        // fuzzyPID.updateSTEPS(90.f);
        // fuzzyPID.calculateDynamic();
        fuzzyPID.calculate();
        once = false;
    }

    //! Run ActionSelector CABSL.
    beginFrame(theFrameInfo->time);
    Cabsl<BalanceActionSelector>::execute(OptionInfos::getOption("ActionRoot"));
    endFrame();

    //! pre footstep action.
    if (action == BalanceActionSelection::footstep && lastAction != BalanceActionSelection::footstep)
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
        const float maxL = 90.f - abs(x);
        // fuzzyPID.updateSTEPS(maxL);
        fuzzyPID.setMaxStep(maxL);
        // fuzzyPID.calculateDynamic();
        // fuzzyPID.printFuzzyTableDynamic();
        printf("[INFO] x: %f xd: %f\n", x, xd);
        // float stepLength = fuzzyPID.getUDynamic(x, xd);
        float stepLength = fuzzyPID.getU(x, xd);
        fuzzyPID.printFuzzyTable();

        //! Set Footstep params
        theFootstepControllerState->stepLength = stepLength;
        theFootstepControllerState->nSteps = 1;
        theFootstepControllerState->leftSwingFirst = (yd >= 0.f ? true : false);
    }

    //! Update last action.
    lastAction = action;
    return action;
}

BalanceActionSelection::BalanceAction BalanceActionSelector::mannualHandle()
{
    //! Configure FuzzyPID
    static bool once = true;
    if (once)
    {
        const float ea = Selector::Ea;
        const float eb = Selector::Eb;
        const float ec = Selector::Ec;
        const float ed = Selector::Ed;
        const float eca = Selector::ECa;
        const float ecb = Selector::ECb;
        const float ecc = Selector::ECc;
        const float ecd = Selector::ECd;

        fuzzyPID.setERange(ea, eb, ec, ed);
        fuzzyPID.setECRange(eca, ecb, ecc, ecd);
        fuzzyPID.calculate();
        once = false;
    }

    if (theKeyStates->pressed[KeyStates::headFront])
    {
        action = BalanceActionSelection::compliance;

        Vector3f comPosition = theFloatingBaseEstimation->WTB.translation();
        comPosition.x() -= theRobotDimensions->leftAnkleToSoleCenter.x();
        const float x = comPosition.x();
        // printf(">\n");
        // printf("[INFO] x: %3.3f\n", x); 
        // printf("----\n\n");
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

        // if (lastAction != BalanceActionSelection::footstep)
        // {
        //     printf(">\n");
        //     printf(" com: %3.3f %3.3f %3.3f\n", comPosition.x(), comPosition.y(), comPosition.z());
        //     printf("comd: %3.3f %3.3f %3.3f\n", comVelocity.x(), comVelocity.y(), comVelocity.z());
        //     printf("step: %3.3f\n", stepLength);
        //     printf("----\n\n");
        // }

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

    //! Update last action.
    lastAction = action;
    return action;
}

bool BalanceActionSelector::comInInitialPosition()
{
    //! (-25.5 +- 1)
    const float ecopxInitial = -25.5f;
    const float bias = 1.f;
    const float min = ecopxInitial - bias;
    const float max = ecopxInitial + bias;
    float ecopx = theCoMProjectionEstimation->estimatedCoP.x();
    if (ecopx > min && ecopx < max)
    {
        return true;
    }
    return false;
}

bool BalanceActionSelector::footstepsAction()
{
    bool footsteps = false;

    float x = theCoMProjectionEstimation->estimatedCoP.x();
    if (abs(x) > 30.f)
    {
        footsteps = true;
    }
    float xd = theFloatingBaseEstimation->comVelocity.x();
    if (abs(xd) > 10.f)
        comVelocityMaxCounter++;
    else
        comVelocityMaxCounter = 0;

    if (comVelocityMaxCounter > 10)
        footsteps = true;

    return footsteps;
}
