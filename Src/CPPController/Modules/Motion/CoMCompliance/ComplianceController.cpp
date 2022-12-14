#include "ComplianceController.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Module/ModuleManager.h"

ComplianceController::ComplianceController()
{
    // covRateThreshold = 10.f * 0.8f;
    errCOV = 0.1f;
    keepJoints = false;
    T = Constants::motionCycleTime;

    Acopx = 0.3f;
    Acopy = 0.3f;

    f_compliance_ecop.open("compliance_ecop.txt", std::ios::out);
}

ComplianceController::~ComplianceController()
{
    f_compliance_ecop.close();
}

void ComplianceController::update()
{
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(BalanceActionSelection);
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(RobotModel);
    UPDATE_REPRESENTATION(FrameInfo);
}

void ComplianceController::update(ComplianceJointRequest &o)
{
    update();

    if (theBalanceActionSelection->targetAction != BalanceActionSelection::compliance)
    {
        return;
    }
    if (!theBalanceTarget->balanceEngineReadyPosture)
    {
        return;
    }

    for (int i = 0; i <= Joints::rAnkleRoll; i++)
    {
        o.angles[i] = theBalanceTarget->lastJointRequest.angles[i];
    }

    float gyroY = theInertialData->gyro.y();

    //! calculate cov
    // float covRateX = 1.f;
    float covRateY = 1.f;
    //! pensulum rotation radius
    float r = theFloatingBaseEstimation->WTB.translation().z();
    float halfSoleX = theRobotDimensions->halfSoleLength;
    float g = Constants::g_1000;
    float a_max = halfSoleX / r * g;
    float a = abs(gyroY) * r / 1000.f / T;
    // float covRate = a / a_max * 10.f;
    // float covRateX = (covRate > 10.f) ? 10.f : covRate;
    const float errCOV_inv = 1.f / errCOV;
    float covRate = a / a_max * errCOV_inv;
    if (covRate < 1.f)
        covRate = 1.f;
    float covRateX = (covRate > errCOV_inv) ? errCOV_inv : covRate;
    float errCOV_X = errCOV * covRateX;

    //! estimated cop x
    float estimatdCopX = theCoMProjectionEstimation->estimatedCoPNormalized.x();
    //! measured cop x
    float measuredCopX = theCoMProjectionEstimation->measuredCoPNormalized.x();

    //! error x and comd
    float errX = theCoMProjectionEstimation->measuredCoPNormalized.x() - theCoMProjectionEstimation->estimatedCoPNormalized.x();
    float comd_x = Acopx * (errX * theCoMProjectionEstimation->normalizedX);

    //! LOG
    const float ecopNX = theCoMProjectionEstimation->estimatedCoPNormalized.x();
    const float mcopNX = theCoMProjectionEstimation->measuredCoPNormalized.x();

    // printf("errCOV_X: %f errX: %f\n", errCOV_X, errX);

    // if (theBalanceTarget->balanceEngineReadyPosture)
    // {
    if ((errCOV_X > abs(errX)))
        comd_x = 0.f;
    f_compliance_ecop << "[" << theFrameInfo->time << "]"
                      << " ecopNX: " << ecopNX << " errCOV: " << errCOV_X << " mcopNX: " << mcopNX << " comdX: " << comd_x << std::endl;
    // }

    //! stop action when large cov
    // if ((errCOV_X > abs(errX)))
    // {
    //     keepJoints = true;
    //     return;
    // }
    // else
    // {
    //     keepJoints = false;
    // }

    //! Get soleLeft target and soleRight target from BalanceTarget.
    Pose3f targetSoleLeft = theBalanceTarget->soleLeftRequest;
    Pose3f targetSoleRight = theBalanceTarget->soleRightRequest;

    targetSoleLeft.translation.x() -= comd_x * dt;
    targetSoleRight.translation.x() -= comd_x * dt;

    theBalanceTarget->soleLeftRequest = targetSoleLeft;
    theBalanceTarget->soleRightRequest = targetSoleRight;

    JointRequest targetRequest;
    bool isPossible = InverseKinematic::calcLegJoints(targetSoleLeft, targetSoleRight, Vector2f::Zero(), targetRequest, *theRobotDimensions);

    //! update LastJointRequest

    if (isPossible)
    {
        theBalanceTarget->lastJointRequest = targetRequest;
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            o.angles[i] = targetRequest.angles[i];
        }
    }

    //! control done flag.
    const float eCoPX = theCoMProjectionEstimation->estimatedCoPNormalized.x();
    if (abs(comd_x) < 1.f && abs(eCoPX) > 0.12)
        // theBalanceTarget->isComplianceControlDone = true;
        complianceFinishedCounter++;
    else
        // theBalanceTarget->isComplianceControlDone = false;
        complianceFinishedCounter = 0;

    if (complianceFinishedCounter > 50)
        theBalanceTarget->isComplianceControlDone = true;
    else
        theBalanceTarget->isComplianceControlDone = false;
    // printf(">\n");
    // printf("normalizedCoP: %f\n", theCoMProjectionEstimation->estimatedCoPNormalized.x());
    // printf("----\n\n");
}

bool ComplianceController::inEstimatedEllipseArea(float estimated, float measured, float cov)
{
    if ((measured < estimated + cov) || (measured > estimated - cov))
        return true;
    else
        return false;
}