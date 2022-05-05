#include "BlackboardThread.h"

// #include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
// #include "Representations/MotionControl/MotionInfo.h"
// #include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"

static thread_local BlackboardThread *theInstance = nullptr;

BlackboardThread::BlackboardThread()
{
    theInstance = this;

    theKeyStatesThread = new RepresentationTemplate<KeyStates>;
    theHeadMotionRequest = new RepresentationTemplate<HeadMotionRequest>;
}

BlackboardThread::~BlackboardThread()
{
    theInstance = nullptr;

    if (theKeyStatesThread != nullptr)
        delete theKeyStatesThread;
    if (theHeadMotionRequest = nullptr)
        delete theHeadMotionRequest;
}

BlackboardThread &BlackboardThread::getInstance()
{
    return *theInstance;
}

void BlackboardThread::setInstance(BlackboardThread *blackboard)
{
    theInstance = blackboard;
}
