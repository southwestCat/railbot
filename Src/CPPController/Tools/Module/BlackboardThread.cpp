#include "BlackboardThread.h"

// #include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
// #include "Representations/MotionControl/MotionInfo.h"
// #include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"

static thread_local BlackboardThread *theInstance = nullptr;

BlackboardThread::BlackboardThread()
{
    theInstance = this;

    theKeyStatesThread = new RepresentationTemplate<KeyStates>;
    theHeadMotionRequest = new RepresentationTemplate<HeadMotionRequest>;
    theHeadMotionEngineOutput = new RepresentationTemplate<HeadMotionEngineOutput>;
}

BlackboardThread::~BlackboardThread()
{
    theInstance = nullptr;

    if (theKeyStatesThread != nullptr)
        delete (RepresentationTemplate<KeyStates> *)theKeyStatesThread;
    if (theHeadMotionRequest != nullptr)
        delete (RepresentationTemplate<HeadMotionRequest> *)theHeadMotionRequest;
    if (theHeadMotionEngineOutput != nullptr)
        delete (RepresentationTemplate<HeadMotionEngineOutput> *)theHeadMotionEngineOutput;
}

BlackboardThread &BlackboardThread::getInstance()
{
    return *theInstance;
}

void BlackboardThread::setInstance(BlackboardThread *blackboard)
{
    theInstance = blackboard;
}
