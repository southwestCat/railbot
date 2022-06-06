#include "ComplianceController.h"
#include "Tools/Module/ModuleManager.h"

void ComplianceController::update()
{
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
}

void ComplianceController::update(ComplianceJointRequest &o)
{
    update();
}
