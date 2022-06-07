#include "ComplianceController.h"
#include "Tools/Module/ModuleManager.h"

ComplianceController::ComplianceController()
{
    gyroThreshold = 0.1f;
    keepJoints = false;
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
    if (abs(gyroY) > gyroThreshold)
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
