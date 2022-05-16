#include "LIPMController.h"
#include "Tools/Module/ModuleManager.h"

void LIPMController::configureOnce()
{
    static bool once = true;
    if (once)
    {
        comVelFilter_.cutoffPeriod(dt_ * 2);
        leftAnkleVelFilter_.cutoffPeriod(dt_ * 2);
        rightAnkleVelFilter_.cutoffPeriod(dt_ * 2);
    }
}

void LIPMController::update()
{
}

void LIPMController::update(StabilizerJointRequest &s)
{
    update();
    run();
}

void LIPMController::run()
{
    netWrenchObs_.update(supportContact());
}

Contact LIPMController::supportContact()
{
    sva::PTransform pose = {Matrix3f::Identity(), {0.f, 0.f, 0.f}};
    return Contact(0.f, 0.f, pose, Contact::SurfaceType::defaultContact);
}
