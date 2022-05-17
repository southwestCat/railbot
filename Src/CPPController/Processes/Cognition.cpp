#include "Cognition.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Tools/Module/BlackboardThread.h"

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
    soccer.execute();
    UPDATE_REPRESENTATION(LEDRequest);
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

    HeadMotionEngineOutput *_theHeadMotionEngineOutput = (HeadMotionEngineOutput *)Blackboard::getInstance().theHeadMotionEngineOutput;
    RepresentationTemplate<HeadMotionEngineOutput> *recvHeadMotionEngineOutput = (RepresentationTemplate<HeadMotionEngineOutput> *)blackboard->theHeadMotionEngineOutput;
    *_theHeadMotionEngineOutput = recvHeadMotionEngineOutput->read();

    MotionInfo *_theMotionInfo = (MotionInfo *)Blackboard::getInstance().theMotionInfo;
    RepresentationTemplate<MotionInfo> *recvMotionInfo = (RepresentationTemplate<MotionInfo> *)blackboard->theMotionInfo;
    *_theMotionInfo = recvMotionInfo->read();
}

void Cognition::send()
{
    HeadMotionRequest *_theHeadMotionRequest = (HeadMotionRequest *)Blackboard::getInstance().theHeadMotionRequest;
    RepresentationTemplate<HeadMotionRequest> *sendHeadMotionRequest = (RepresentationTemplate<HeadMotionRequest> *)blackboard->theHeadMotionRequest;
    sendHeadMotionRequest->write(*_theHeadMotionRequest);

    MotionRequest *_theMotionRequest = (MotionRequest *)Blackboard::getInstance().theMotionRequest;
    RepresentationTemplate<MotionRequest> *sendMotionRequest = (RepresentationTemplate<MotionRequest> *)blackboard->theMotionRequest;
    sendMotionRequest->write(*_theMotionRequest);

    LEDRequest *_theLEDRequest = (LEDRequest *)Blackboard::getInstance().theLEDRequest;
    RepresentationTemplate<LEDRequest> *sendLEDRequest = (RepresentationTemplate<LEDRequest> *)blackboard->theLEDRequest;
    sendLEDRequest->write(*_theLEDRequest);
}