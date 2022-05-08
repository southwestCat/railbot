#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Tools/Module/Blackboard.h"

class MotionSelectorBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(MotionRequest);

public:
    MotionSelectorBase()
    {
        interpolationTimes.fill(300.f);
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