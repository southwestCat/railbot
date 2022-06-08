#include "ComplianceController.h"
#include "Tools/Module/ModuleManager.h"

ComplianceController::ComplianceController()
{
    covRateThreshold = 10.f * 0.8f;
    errCOV = 0.1f;
    keepJoints = false;
    T = Constants::motionCycleTime;
}

void ComplianceController::update()
{
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(BalanceActionSelection);
    UPDATE_REPRESENTATION(JointAngles);
}

void ComplianceController::update(ComplianceJointRequest &o)
{
    update();

    if (theBalanceActionSelection->targetAction != BalanceActionSelection::compliance)
    {
        return;
    }

    float gyroY = theInertialData->gyro.y();
    // if (abs(gyroY) > gyroThreshold)
    // {
    //     if (!keepJoints)
    //     {
    //         for (int i = Joints::firstLegJoint; i < Joints::rAnkleRoll; i++)
    //         {
    //             o.angles[i] = theJointAngles->angles[i];
    //         }
    //     }
    //     keepJoints = true;
    //     return;
    // }
    // else
    // {
    //     keepJoints = false;
    // }

    //! calculate cov
    // float covRateX = 1.f;
    float covRateY = 1.f;
    //! pensulum rotation radius
    float r = theFloatingBaseEstimation->WTB.translation().z();
    float halfSoleX = theRobotDimensions->halfSoleLength;
    float g = Constants::g_1000;
    float a_max = halfSoleX / r * g;
    float a = abs(gyroY) * r / 1000.f / T;
    float covRate = a / a_max * 10.f;
    float covRateX = (covRate > 10.f) ? 10.f : covRate;
    float errCOV_X = errCOV * covRateX;

    //! stop action when large cov
    if (covRateX > covRateThreshold)
    {
        if (!keepJoints)
        {
            for (int i = Joints::firstLegJoint; i < Joints::rAnkleRoll; i++)
            {
                o.angles[i] = theJointAngles->angles[i];
            }
        }
        keepJoints = true;
        return;
    }
    else
    {
        keepJoints = false;
    }

    float errX = theCoMProjectionEstimation->measuredCoPNormalized.x() - theCoMProjectionEstimation->estimatedCoPNormalized.x();
}
