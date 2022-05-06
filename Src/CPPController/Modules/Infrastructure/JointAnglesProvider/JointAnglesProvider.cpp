#include "JointAnglesProvider.h"
#include "Tools/Module/ModuleManager.h"

void JointAnglesProvider::update()
{
    UPDATE_REPRESENTATION(JointSensorData);
}

void JointAnglesProvider::update(JointAngles &j)
{
    update();
    j = *theJointSensorData;
}
