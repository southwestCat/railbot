#include "MotionCombinator.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"

void MotionCombinator::update()
{
    UPDATE_REPRESENTATION(HeadJointRequest);
    UPDATE_REPRESENTATION(StiffnessSettings);
}

void MotionCombinator::update(JointRequest &jointRequest)
{
    update();
    MotionUtilities::copy(*theHeadJointRequest, jointRequest, *theStiffnessSettings, Joints::headYaw, Joints::headPitch);

    
}

void MotionCombinator::update(MotionInfo &motionInfo)
{
    update();
    motionInfo = this->motionInfo;
}
