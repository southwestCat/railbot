#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Configuration//RobotDimensions.h"
#include "Representations/MotionControl/SitDownEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Tools/Module/Blackboard.h"

class SitDownEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(LegMotionSelection);

    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(StiffnessSettings);

    const int interpolateTime = 3000;

    unsigned startTime = 0;
    unsigned nowTime = 0;
};

class SitDownEngine : public SitDownEngineBase
{
public:
    SitDownEngine();
    void update(SitDownEngineOutput &s);

private:
    void update();
    JointRequest targetJoint;
    JointRequest startJoint;
};
