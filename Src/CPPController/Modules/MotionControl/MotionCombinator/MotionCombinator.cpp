#include "MotionCombinator.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"

void MotionCombinator::update()
{
    UPDATE_REPRESENTATION(HeadJointRequest);
}

void MotionCombinator::update(JointRequest &jointRequest)
{
    update();
    MotionUtilities::copy(*theHeadJointRequest, jointRequest, *theStiffnessSettings, Joints::headYaw, Joints::headPitch);
}
