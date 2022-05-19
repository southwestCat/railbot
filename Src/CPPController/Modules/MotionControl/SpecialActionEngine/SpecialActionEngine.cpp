#include "SpecialActionEngine.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Modules/Motion/MotionConfigure.h"

void SpecialActionEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
}

void SpecialActionEngine::update(SpecialActionEngineOutput &s)
{
    update();
    //! Play dead.
    if (theLegMotionSelection->targetMotion != MotionRequest::specialAction)
    {
        reset(s);
        return;
    }
    switch (theMotionRequest->specialActionRequest.specialAction)
    {
    case SpecialActionRequest::playDead:
        reset(s);
        playDead(s);
        break;
    case SpecialActionRequest::stand:
        stand(s);
        break;
    default:
        reset(s);
        playDead(s);
        printf("[WARNING] Unknown special action ID.\n");
        break;
    }
}

void SpecialActionEngine::reset(SpecialActionEngineOutput &s)
{
    s.isLeavingPossible = false;
    startJoints_ = *theJointAngles;
    startTime_ = theFrameInfo->time;
}

void SpecialActionEngine::playDead(SpecialActionEngineOutput &s)
{
    s.angles.fill(JointAngles::off);
    s.isLeavingPossible = true;
}

void SpecialActionEngine::stand(SpecialActionEngineOutput &s)
{
    int nowTime = theFrameInfo->getTimeSince(startTime_);
    float ratio = (float)nowTime / (float)standInterpolateTime;
    if (ratio > 1)
    {
        s.isLeavingPossible = true;
        return;
    }
    s.isLeavingPossible = false;
    JointRequest targetJointRequest;
    JointRequest j;

    float target = MotionConfig::hipHeight;
    Pose3f targetL = Pose3f(Vector3f(0.f, theRobotDimensions->yHipOffset, -target));
    Pose3f targetR = Pose3f(Vector3f(0.f, -theRobotDimensions->yHipOffset, -target));
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), targetJointRequest, *theRobotDimensions);
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        j.angles[i] = startJoints_.angles[i] + ratio * (targetJointRequest.angles[i] - startJoints_.angles[i]);
    }
    if (isPossible)
    {
        MotionUtilities::copy(j, s, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
    }
}
