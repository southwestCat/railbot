#include "StandEngine.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Module/ModuleManager.h"

StandEngine::StandEngine()
{
    MotionUtilities::stand(targetJoint);
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