#include <iostream>
#include "Motion.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
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
    receive();
}

void Motion::updateModules()
{
    UPDATE_REPRESENTATION(KeyStates);
    UPDATE_REPRESENTATION(JointRequest);

    JointRequest *j = (JointRequest *)Blackboard::getInstance().theJointRequest;
    // std::cout << j->angles[Joints::headYaw] << " " << j->stiffnessData.stiffnesses[Joints::headYaw] << std::endl;
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

    HeadMotionEngineOutput *_theHeadMotionEngineOutput = (HeadMotionEngineOutput *)Blackboard::getInstance().theHeadMotionEngineOutput;
    RepresentationTemplate<HeadMotionEngineOutput> *_recvHeadMotionEngineOutput = (RepresentationTemplate<HeadMotionEngineOutput> *)blackboard->theHeadMotionEngineOutput;
    _recvHeadMotionEngineOutput->write(*_theHeadMotionEngineOutput);
}

void Motion::receive()
{
    HeadMotionRequest *_theHeadMotionRequest = (HeadMotionRequest *)Blackboard::getInstance().theHeadMotionRequest;
    RepresentationTemplate<HeadMotionRequest> *recvHeadMotionRequest = (RepresentationTemplate<HeadMotionRequest> *)blackboard->theHeadMotionRequest;
    *_theHeadMotionRequest = recvHeadMotionRequest->read();
}
