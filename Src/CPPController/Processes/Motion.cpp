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
    FrameInfo *_theFrameInfo = (FrameInfo *)Blackboard::getInstance().theFrameInfo;
    NaoProvider::getInstance().update(*_theFrameInfo);
}

void Motion::afterModules()
{
}

void Motion::afterFrame()
{
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
