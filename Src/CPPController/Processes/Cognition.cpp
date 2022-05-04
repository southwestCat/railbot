#include "Cognition.h"
#include "Tools/Module/BlackboardThread.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

void Cognition::tick()
{
    beforeFrame();
    beforeModules();
    updateModules();
    afterModules();
    afterFrame();
}

void Cognition::beforeFrame()
{
}

void Cognition::beforeModules()
{
    resetUpdate();
    receive();
}

void Cognition::updateModules()
{
}

void Cognition::afterModules()
{
    // TODO: Generate SPLMessage
    // TODO: send Data to other thread
    send();
}

void Cognition::afterFrame()
{
    // TODO: If there is already a frame waiting to be processed, do not wait for the next one to arrive.
    // return !acceptNext;
}

void Cognition::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedRepresentation;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}

void Cognition::receive()
{
    KeyStates *_theKeyStates = (KeyStates *)Blackboard::getInstance().theKeyStates;
    RepresentationTemplate<KeyStates> *recvKeyStates = (RepresentationTemplate<KeyStates> *)blackboard->theKeyStatesThread;
    *_theKeyStates = recvKeyStates->read();
}

void Cognition::send()
{
}