#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/SitDownEngineOutput.h"
#include "Representations/MotionControl/BalanceEngineOutput.h"
#include "Tools/Module/Blackboard.h"

class MotionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(SpecialActionEngineOutput);
    REQUIRES_REPRESENTATION(StandEngineOuptut);
    REQUIRES_REPRESENTATION(SitDownEngineOutput);
    REQUIRES_REPRESENTATION(BalanceEngineOutput);

    USES_REPRESENTATION(MotionRequest);

public:
    MotionSelectorBase()
    {
        interpolationTimes.fill(100.f);
    }

    int playDeadDelay = 2000;
    std::array<int, MotionRequest::numOfMotions> interpolationTimes;
};

class MotionSelector : public MotionSelectorBase
{
public:
    void update(LegMotionSelection &legMotionSelection);

private:
    unsigned lastExecution = 0;
    unsigned lastExecutionArm = 0;
    unsigned lastExecutionLeg = 0;

    MotionRequest::Motion lastLegMotion = MotionRequest::specialAction;

    void update();
    void interpolate(float *ratios, const int amount, const int interpolationTime, const int targetMotion);
};