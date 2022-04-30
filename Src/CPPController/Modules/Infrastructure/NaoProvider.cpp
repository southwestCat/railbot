#include "NaoProvider.h"
#include "SharedMemoryData.h"

#include <cassert>

thread_local NaoProvider *NaoProvider::theInstance = nullptr;

NaoProvider::~NaoProvider()
{
    NaoProvider::theInstance = nullptr;
}

NaoProvider::NaoProvider()
{
    NaoProvider::theInstance = this;
}

void NaoProvider::setInstance(NaoProvider *instance)
{
    theInstance = instance;
}

NaoProvider &NaoProvider::getInstance()
{
    return *theInstance;
}

void NaoProvider::finishFrame()
{
    if (theInstance)
    {
        theInstance->send();
    }
}

void NaoProvider::waitForFrameData()
{
    if (theInstance)
    {
        theInstance->naoBody.wait();
    }
}

void NaoProvider::send()
{
    float *actuators;
    naoBody.openActuators(actuators);
    int j = 0;
    assert(headYawPositionActuator == 0);
    assert(static_cast<int>(Joints::numOfJoints) - 1 == numOfPositionActuatorIds); // rHipYawPitch missin lbh
    for (int i = 0; i < Joints::numOfJoints; ++i, ++j)
    {
        if (i == Joints::rHipYawPitch) // missing on Nao
        {
            ++i;
        }
        if (theJointRequest->angles[i] == SensorData::off)
        {
            actuators[j] = 0.f;
            actuators[j + numOfPositionActuatorIds] = 0.f; // stiffness
        }
        else
        {
            actuators[j + numOfPositionActuatorIds] = static_cast<float>(theJointRequest->stiffnessData.stiffnesses[i]) / 100.f;
        }
    }
}
