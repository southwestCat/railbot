#include "CoMProjectionObserver.h"
#include "Tools/Module/ModuleManager.h"

void CoMProjectionObserver::update()
{
    UPDATE_REPRESENTATION(FsrSensorData);
    UPDATE_REPRESENTATION(RobotModel);
}

void CoMProjectionObserver::update(CoMProjectionEstimation &o)
{
    update();

    Vector3f com = theRobotModel->centerOfMass;
    o.estimatedCoP = {com.x(), com.y()};
    Vector3f cop = theNetWrenchEstimation->netZMP;
    o.measuredCoP = {cop.x(), cop.y()};
}
