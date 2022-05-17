#include "BalanceEngine.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"

void BalanceEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(StabilizerJointRequest);
}

void BalanceEngine::update(BalanceEngineOutput &o)
{
    update();

    if (theLegMotionSelection->targetMotion != MotionRequest::balance)
    {
        o.isLeavingPossible = false;
        startTime = theFrameInfo->time;
        return;
    }
    o.isLeavingPossible = true;
    nowTime = theFrameInfo->getTimeSince(startTime);

    MotionUtilities::copy(*theStabilizerJointRequest, o, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
}