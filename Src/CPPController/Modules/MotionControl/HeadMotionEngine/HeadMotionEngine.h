#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Module/Blackboard.h"

class HeadMotionEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(HeadMotionRequest);
    REQUIRES_REPRESENTATION(HeadLimits);
    REQUIRES_REPRESENTATION(JointAngles);
};

class HeadMotionEngine : public HeadMotionEngineBase
{
public:
    HeadMotionEngine();

    void update();
    void update(HeadMotionEngineOutput &headMotionEngineOutput);

private:
    Angle requestedPan;
    Angle requestedTilt;
    Vector2f lastSpeed;
};