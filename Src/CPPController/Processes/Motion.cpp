#include "Motion.h"
#include <iostream>

void Motion::tick()
{
    beforeFrame();
    beforeModules();
    updateModules();
    afterModules();
    afterFrame();
}

void Motion::beforeFrame()
{
}

void Motion::beforeModules()
{
    resetUpdate();
}

void Motion::updateModules()
{
    JointRequest *j = (JointRequest *)Blackboard::getInstance().theJointRequest;
    j->angles[Joints::headYaw] = 0_deg;
    j->stiffnessData.stiffnesses[Joints::headYaw] = 50;
}

void Motion::afterModules()
{
    NaoProvider::finishFrame();
}

void Motion::afterFrame()
{
    if (Blackboard::getInstance().exists(CLASS2STRING(JointSensorData)))
    {
        NaoProvider::waitForFrameData();
    }
}

void Motion::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedRepresentation;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}
