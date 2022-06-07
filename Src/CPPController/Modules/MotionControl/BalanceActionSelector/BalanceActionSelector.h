#pragma once

#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Tools/Module/Blackboard.h"

class BalanceActionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);

    USES_REPRESENTATION(KeyStates);
};

class BalanceActionSelector : public BalanceActionSelectorBase
{
public:
    void update(BalanceActionSelection &b);

private:
    void update();
};
