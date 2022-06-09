#include "BalanceEngine.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Motion/InverseKinematic.h"

void BalanceEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(StabilizerJointRequest);
    UPDATE_REPRESENTATION(ComplianceJointRequest);
    UPDATE_REPRESENTATION(FootstepJointRequest);
    UPDATE_REPRESENTATION(BalanceActionSelection);
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
    UPDATE_REPRESENTATION(RobotModel);
}

void BalanceEngine::update(BalanceEngineOutput &o)
{
    update();

    if (theLegMotionSelection->targetMotion != MotionRequest::balance)
    {
        o.isLeavingPossible = false;
        startTime = theFrameInfo->time;
        float soleL = theRobotModel->soleLeft.translation.z();
        float soleR = theRobotModel->soleRight.translation.z();
        initHeight = abs((soleL + soleR) / 2.f);

        startJoints_ = *theJointAngles;

        return;
    }
    o.isLeavingPossible = true;

    if (!readyPosture(o))
        return;

    const JointRequest *jointRequest[BalanceActionSelection::numOfBalanceAction];
    jointRequest[BalanceActionSelection::compliance] = theComplianceJointRequest;
    jointRequest[BalanceActionSelection::dcm] = theStabilizerJointRequest;
    jointRequest[BalanceActionSelection::footstep] = theFootstepJointRequest;

    // //! Balance action selection;
    BalanceActionSelection::BalanceAction target = theBalanceActionSelection->targetAction;

    if (target == BalanceActionSelection::footstep)
    {
        // theMPCControllerState->comPosition = theFloatingBaseEstimation.
    }

    MotionUtilities::copy(*jointRequest[target], o, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
}

bool BalanceEngine::readyPosture(BalanceEngineOutput &o)
{
    unsigned nowTime = theFrameInfo->time - startTime;
    if (nowTime > readyPostureTime + readyWaitTime)
    {
        return true;
    }
    else if (nowTime > readyPostureTime)
    {
        return false;
    }

    float t = (float)nowTime / 1000.f;
    const float duringT = (readyPostureTime) / 1000.f;
    float target = hipHeight;
    Pose3f targetL = Pose3f(Vector3f(0.f, theRobotDimensions->yHipOffset, -target));
    Pose3f targetR = Pose3f(Vector3f(0.f, -theRobotDimensions->yHipOffset, -target));
    //! update request soleL and soleR to BalanceTarget.
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;
    
    JointRequest targetJointRequest_;
    JointRequest jointRequest_;
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), targetJointRequest_, *theRobotDimensions);
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        jointRequest_.angles[i] = startJoints_.angles[i] + t / duringT * (targetJointRequest_.angles[i] - startJoints_.angles[i]);
    }
    //! update lastJointRequest to BalanceTarget.
    theBalanceTarget->lastJointRequest = jointRequest_;
    if (isPossible)
    {
        MotionUtilities::copy(jointRequest_, o, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
    }
    return false;
}