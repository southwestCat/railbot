#include "BalanceActionSelector.h"
#include "Tools/Module/ModuleManager.h"
#include <iostream>

void BalanceActionSelector::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
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
        o.targetAction = BalanceActionSelection::dcm;
    }
    if (theKeyStates->pressed[KeyStates::headRear])
    {
        o.targetAction = BalanceActionSelection::footstep;
    }
    std::cout << o.targetAction << std::endl;
}
