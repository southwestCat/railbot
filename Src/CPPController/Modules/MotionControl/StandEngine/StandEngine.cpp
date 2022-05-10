#include "StandEngine.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Module/ModuleManager.h"

StandEngine::StandEngine()
{
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        targetJoint.angles[i] = 0_deg;
    }
    targetJoint.angles[Joints::lHipPitch] = -3_deg;
    targetJoint.angles[Joints::rHipPitch] = -3_deg;
}

void StandEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
}

void StandEngine::update(StandEngineOuptut &s)
{
    update();

    if (theLegMotionSelection->targetMotion != MotionRequest::stand)
    {
        s.isLeavingPossible = false;
        startTime = theFrameInfo->time;
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            startJoint.angles[i] = theJointAngles->angles[i];
        }
        return;
    }

    nowTime = theFrameInfo->getTimeSince(startTime);
    float ratio = (float)nowTime / (float)interpolateTime;

    if (ratio > 1)
    {
        s.isLeavingPossible = true;
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            s.angles[i] = targetJoint.angles[i];
        }
    }
    else
    {
        s.isLeavingPossible = false;
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            s.angles[i] = startJoint.angles[i] + ratio * (targetJoint.angles[i] - startJoint.angles[i]);
        }
    }
}