#include "LegMotionCombinator.h"
#include "Tools/Module/ModuleManager.h"

void LegMotionCombinator::update()
{
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(LegMotionSelection);
}

void LegMotionCombinator::update(LegJointRequest &legJointRequest)
{
    update();

    const JointRequest *jointRequest[MotionRequest::numOfMotions];
    // jointRequest[MotionRequest::walk] = 
}
