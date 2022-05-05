#include <iostream>
#include "Motion.h"
#include "Tools/Module/BlackboardThread.h"

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
    UPDATE_REPRESENTATION(KeyStates);
    UPDATE_REPRESENTATION(JointRequest);
}

void Motion::afterModules()
{
    send();
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

void Motion::send()
{
    KeyStates *_theKeyStates = (KeyStates *)Blackboard::getInstance().theKeyStates;
    RepresentationTemplate<KeyStates> *recvKeyStates = (RepresentationTemplate<KeyStates> *)blackboard->theKeyStatesThread;
    recvKeyStates->write(*_theKeyStates);
}
