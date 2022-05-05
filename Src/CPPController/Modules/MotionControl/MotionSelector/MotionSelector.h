#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Module/Blackboard.h"

class MotionSelectorBase
{
public:
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
    
private:
    void update();
};