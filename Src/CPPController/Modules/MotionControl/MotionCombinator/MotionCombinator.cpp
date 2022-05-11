#include "MotionCombinator.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"

void MotionCombinator::update()
{
    UPDATE_REPRESENTATION(HeadJointRequest);
    UPDATE_REPRESENTATION(LegJointRequest);
    UPDATE_REPRESENTATION(LegMotionSelection);
    UPDATE_REPRESENTATION(StandEngineOuptut);
    UPDATE_REPRESENTATION(SitDownEngineOutput);
}

void MotionCombinator::update(JointRequest &jointRequest)
{
    update();
    MotionUtilities::copy(*theHeadJointRequest, jointRequest, *theStiffnessSettings, Joints::headYaw, Joints::headPitch);
    MotionUtilities::copy(*theLegJointRequest, jointRequest, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
    assert(jointRequest.isValid());

    for (int i = 0; i < Joints::numOfJoints; i++)
    {
        if (jointRequest.angles[i] != JointAngles::off && jointRequest.angles[i] != JointAngles::ignore)
            theJointLimits->limits[i].clamp(jointRequest.angles[i]);
        if (theLegMotionSelection->targetMotion == MotionRequest::stand)
        {
            if (theStandEngineOuptut->isLeavingPossible)
            {
                motionInfo.motion = MotionRequest::stand;
            }
        }
        else if (theLegMotionSelection->targetMotion == MotionRequest::sitDown)
        {
            if (theSitDownEngineOutput->isLeavingPossible)
            {
                motionInfo.motion = MotionRequest::sitDown;
            }
        }
        else if (theLegMotionSelection->ratios[theLegMotionSelection->targetMotion] == 1.f)
        {
            motionInfo.motion = theLegMotionSelection->targetMotion;
        }
    }
}

void MotionCombinator::update(MotionInfo &motionInfo)
{
    update();
    motionInfo = this->motionInfo;
}
