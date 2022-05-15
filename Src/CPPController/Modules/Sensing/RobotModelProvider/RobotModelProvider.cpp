#include "RobotModelProvider.h"
#include "Tools/Module/ModuleManager.h"

void RobotModelProvider::update()
{
    UPDATE_REPRESENTATION(JointAngles);
}

void RobotModelProvider::update(RobotModel &r)
{
    update();

    r.setJointData(*theJointAngles, *theRobotDimensions, *theMassCalibration);
}
