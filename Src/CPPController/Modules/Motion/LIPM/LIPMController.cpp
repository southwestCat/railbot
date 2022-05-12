#include "LIPMController.h"
#include "Tools/Module/ModuleManager.h"

void LIPMController::update()
{
    UPDATE_REPRESENTATION(NetWrenchEstimation);
}

void LIPMController::update(StabilizerJointRequest &s)
{
    update();
}
