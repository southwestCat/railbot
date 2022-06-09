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

    if (theKeyStates->pressed[KeyStates::headFront])
    {
        o.targetAction = BalanceActionSelection::compliance;
    }
    if (theKeyStates->pressed[KeyStates::headMiddle])
    {
        o.targetAction = BalanceActionSelection::footstep;
    }
    if (theKeyStates->pressed[KeyStates::headRear])
    {
        o.targetAction = BalanceActionSelection::dcm;
    }

    if (theCoMProjectionEstimation->measuredCoPNormalized.x() > 0.8)
    {
        // std::cout << "Footsteps front.\n";
    }
    else if (theCoMProjectionEstimation->measuredCoPNormalized.x() < -0.8)
    {
        // std::cout << "Footsteps back.\n";
    }
    else
    {
        // std::cout << "Compliance.\n";
    }
    // std::cout << o.targetAction << std::endl;
    // std::cout << theRobotModel->centerOfMass.transpose() << std::endl;
}
