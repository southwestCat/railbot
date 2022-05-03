#include "NaoProvider.h"
#include "SharedMemoryData.h"
#include "Platform/Time.h"

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
    j += numOfPositionActuatorIds;
    assert(j == faceLedRedLeft0DegActuator);

    naoBody.closeActuators();
}

void NaoProvider::update()
{
}

void NaoProvider::update(FrameInfo &frameInfo)
{
    frameInfo.time = std::max(frameInfo.time + 1, Time::getCurrentSystemTime());
}

void NaoProvider::update(InertialSensorData &inertialSensorData)
{
    float *sensors = naoBody.getSensors();

    // TODO:
    inertialSensorData.gyro.x() = sensors[gyroXSensor];
    inertialSensorData.gyro.y() = sensors[gyroYSensor];
    inertialSensorData.gyro.z() = sensors[gyroZSensor]; // Aldebarans z-gyron is negated in V5, but in V6 it is positive.

    inertialSensorData.acc.x() = sensors[accXSensor];
    inertialSensorData.acc.y() = sensors[accYSensor];
    inertialSensorData.acc.z() = sensors[accZSensor];
    printf("%f\n\n", sensors[accZSensor]);

    inertialSensorData.angle.x() = sensors[angleXSensor];
    inertialSensorData.angle.y() = sensors[angleYSensor];
    inertialSensorData.angle.z() = sensors[angleZSensor]; // TODO:
}
