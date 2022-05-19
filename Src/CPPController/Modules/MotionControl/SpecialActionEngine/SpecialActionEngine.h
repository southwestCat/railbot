#pragma once

#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Tools/Module/Blackboard.h"

class SpecialActionEngineBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(JointAngles);

    USES_REPRESENTATION(MotionRequest);
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(LegMotionSelection);
    USES_REPRESENTATION(StiffnessSettings);
};

class SpecialActionEngine : public SpecialActionEngineBase
{
public:
    void update(SpecialActionEngineOutput &s);
    void stand(SpecialActionEngineOutput &s);
    void playDead(SpecialActionEngineOutput &s);

private:
    void update();
    void reset(SpecialActionEngineOutput &s);

    unsigned startTime_ = 0;
    const int standInterpolateTime = 2000; 
    JointAngles startJoints_;
};
