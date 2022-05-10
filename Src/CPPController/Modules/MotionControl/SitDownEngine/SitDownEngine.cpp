#include "SitDownEngine.h"
#include "Tools/Module/ModuleManager.h"

// 0.0    0.0  -50.0  124.0  -67.8    0.0    0.0    0.0  -50.0  124.0  -67.8    0.0
SitDownEngine::SitDownEngine()
{
    targetJoint.angles[Joints::lHipYawPitch] = 0_deg;
    targetJoint.angles[Joints::lHipRoll] = 0_deg;
    targetJoint.angles[Joints::lHipPitch] = -50_deg;
    targetJoint.angles[Joints::lKneePitch] = 124_deg;
    targetJoint.angles[Joints::lAnklePitch] = -67.8_deg;
    targetJoint.angles[Joints::lAnkleRoll] = 0_deg;
    targetJoint.angles[Joints::lHipYawPitch] = 0_deg;
    targetJoint.angles[Joints::rHipRoll] = 0_deg;
    targetJoint.angles[Joints::rHipPitch] = -50_deg;
    targetJoint.angles[Joints::rKneePitch] = 124_deg;
    targetJoint.angles[Joints::rAnklePitch] = -67.8_deg;
    targetJoint.angles[Joints::rAnkleRoll] = 0_deg;
}

void SitDownEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(LegMotionSelection);
}

void SitDownEngine::update(SitDownEngineOutput &s)
{
    update();

    if (theLegMotionSelection->targetMotion != MotionRequest::sitDown)
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
            s.angles[i] = JointAngles::off;
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