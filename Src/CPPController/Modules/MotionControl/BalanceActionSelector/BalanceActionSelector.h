#pragma once

#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/Module/Blackboard.h"

class BalanceActionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(KeyStates);
};

class BalanceActionSelector : public BalanceActionSelectorBase
{
public:
    void update(BalanceActionSelection &b);

private:
    void update();
};
