#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Configuration//RobotDimensions.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Tools/Module/Blackboard.h"

class StandEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);
    
    USES_REPRESENTATION(MotionRequest);
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(StiffnessSettings);

    const int interpolateTime = 2000;

    unsigned startTime = 0;
    unsigned nowTime = 0;
};

class StandEngine : public StandEngineBase
{
public:
    void update(StandEngineOuptut &standEngineOuptut);
    StandEngine();

private:
    void update();
    JointRequest targetJoint;
    JointRequest startJoint;
};