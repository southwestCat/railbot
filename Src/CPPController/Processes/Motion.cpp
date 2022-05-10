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
    UPDATE_REPRESENTATION(MotionInfo);

    // test();
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
    RepresentationTemplate<KeyStates> *sendKeyStates = (RepresentationTemplate<KeyStates> *)blackboard->theKeyStatesThread;
    sendKeyStates->write(*_theKeyStates);

    HeadMotionEngineOutput *_theHeadMotionEngineOutput = (HeadMotionEngineOutput *)Blackboard::getInstance().theHeadMotionEngineOutput;
    RepresentationTemplate<HeadMotionEngineOutput> *sendHeadMotionEngineOutput = (RepresentationTemplate<HeadMotionEngineOutput> *)blackboard->theHeadMotionEngineOutput;
    sendHeadMotionEngineOutput->write(*_theHeadMotionEngineOutput);

    MotionInfo *_theMotionInfo = (MotionInfo *)Blackboard::getInstance().theMotionInfo;
    RepresentationTemplate<MotionInfo> *sendMotionInfo = (RepresentationTemplate<MotionInfo> *)blackboard->theMotionInfo;
    sendMotionInfo->write(*_theMotionInfo);
}

void Motion::receive()
{
    HeadMotionRequest *_theHeadMotionRequest = (HeadMotionRequest *)Blackboard::getInstance().theHeadMotionRequest;
    RepresentationTemplate<HeadMotionRequest> *recvHeadMotionRequest = (RepresentationTemplate<HeadMotionRequest> *)blackboard->theHeadMotionRequest;
    *_theHeadMotionRequest = recvHeadMotionRequest->read();

    MotionRequest *_theMotionRequest = (MotionRequest *)Blackboard::getInstance().theMotionRequest;
    RepresentationTemplate<MotionRequest> *recvMotionRequest = (RepresentationTemplate<MotionRequest> *)blackboard->theMotionRequest;
    *_theMotionRequest = recvMotionRequest->read();
}

void Motion::test()
{
    // FrameInfo *_theFrameInfo = (FrameInfo *)Blackboard::getInstance().theFrameInfo;
    // static unsigned startTime = _theFrameInfo->time;
    // unsigned nowTime = _theFrameInfo->getTimeSince(startTime);
    // float headYawAngle = 10_deg * sin(2*pi/4000.0 * (float)nowTime);
    // JointRequest *j = (JointRequest *)Blackboard::getInstance().theJointRequest;
    // j->angles[Joints::headYaw] = headYawAngle;
    // j->stiffnessData.stiffnesses[Joints::headYaw] = 50;


    // JointRequest *j = (JointRequest *)Blackboard::getInstance().theJointRequest;
    // j->angles[Joints::headYaw] = -30_deg;
    // j->stiffnessData.stiffnesses[Joints::headYaw] = 20;
}
