#include "NaoProvider.h"
#include "SharedMemoryData.h"
#include "Platform/Time.h"
#include "Tools/Module/ModuleManager.h"

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
            actuators[j] = theJointRequest->angles[i];
            actuators[j + numOfPositionActuatorIds] = static_cast<float>(theJointRequest->stiffnessData.stiffnesses[i]) / 100.f;
        }
    }
    j += numOfPositionActuatorIds;
    assert(j == faceLedRedLeft0DegActuator);

    const LEDRequest &ledRequest(*theLEDRequest);
    bool on = (theFrameInfo->time / 50 & 8) != 0;
    bool fastOn = (theFrameInfo->time / 10 & 8) != 0;
    for (int i = 0; i < LEDRequest::numOfLEDs; ++i)
    {
        actuators[j++] = (ledRequest.ledStates[i] == LEDRequest::on || (ledRequest.ledStates[i] == LEDRequest::blinking && on) || (ledRequest.ledStates[i] == LEDRequest::fastBlinking && fastOn)) ? 1.0f : (ledRequest.ledStates[i] == LEDRequest::half ? 0.5f : 0.0f);
    }

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

    inertialSensorData.angle.x() = sensors[angleXSensor];
    inertialSensorData.angle.y() = sensors[angleYSensor];
    inertialSensorData.angle.z() = sensors[angleZSensor]; // TODO:
}

void NaoProvider::update(FsrSensorData &fsrSensorData)
{
    float *sensors = naoBody.getSensors();
    for (int leg = 0; leg < Legs::numOfLegs; leg++)
    {
        for (int sensor = 0; sensor < FsrSensors::numOfFsrSensors; sensor++)
        {
            fsrSensorData.pressures[leg][sensor] = sensors[lFSRFrontLeftSensor + leg * FsrSensors::numOfFsrSensors + sensor];
        }
        fsrSensorData.totals[leg] = sensors[lFSRTotalSensor + leg];
    }

    float lfl = fsrSensorData.pressures[Legs::left][FsrSensors::fl];
    float lfr = fsrSensorData.pressures[Legs::left][FsrSensors::fr];
    float lbl = fsrSensorData.pressures[Legs::left][FsrSensors::bl];
    float lbr = fsrSensorData.pressures[Legs::left][FsrSensors::br];

    float rfl = fsrSensorData.pressures[Legs::right][FsrSensors::fl];
    float rfr = fsrSensorData.pressures[Legs::right][FsrSensors::fr];
    float rbl = fsrSensorData.pressures[Legs::right][FsrSensors::bl];
    float rbr = fsrSensorData.pressures[Legs::right][FsrSensors::br];

    printf("----------foot sensors----------\n");
    printf("   left       right\n");
    printf("+--------+ +--------+\n");
    printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  front\n", lfl, lfr, lbl, lbr);
    printf("|        | |        |\n");
    printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  back\n", rfl, rfr, rbl, rbr);
    printf("+--------+ +--------+\n");
}

void NaoProvider::update(JointSensorData &jointSensorData)
{
    UPDATE_REPRESENTATION(FrameInfo);

    float *sensors = naoBody.getSensors();
    int j = 0;
    for (int i = 0; i < Joints::numOfJoints; i++)
    {
        if (i == Joints::rHipYawPitch)
        {
            jointSensorData.angles[i] = jointSensorData.angles[Joints::lHipYawPitch];
            jointSensorData.currents[i] = jointSensorData.currents[Joints::lHipYawPitch];
            jointSensorData.temperatures[i] = jointSensorData.temperatures[Joints::lHipYawPitch];
            jointSensorData.status[i] = jointSensorData.status[Joints::lHipYawPitch];
        }
        else
        {
            // jointSensorData.angles[i] = sensors[j++] - theJointCalibration.offsets[i];
            jointSensorData.angles[i] = sensors[j++];
            jointSensorData.currents[i] = static_cast<short>(1000.f * sensors[j++]);
            jointSensorData.temperatures[i] = static_cast<unsigned char>(sensors[j++]);
            // jointSensorData.status[i] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int *>(&sensors[j++]));
        }
    }
    jointSensorData.timestamp = theFrameInfo->time;
}

void NaoProvider::update(KeyStates &keyStates)
{
    float *sensors = naoBody.getSensors();

    for (int i = 0, j = headTouchFrontSensor; i < KeyStates::numOfKeys; i++, j++)
    {
        keyStates.pressed[i] = sensors[j] != 0;
    }
}

void NaoProvider::update(SystemSensorData &systemSensorData)
{
    UPDATE_REPRESENTATION(FrameInfo);

    float *sensors = naoBody.getSensors();

    if (theFrameInfo->getTimeSince(lastBodyTemperatureReadTime) * 1000 > 10)
    {
        lastBodyTemperatureReadTime = theFrameInfo->time;
        systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
    }
    systemSensorData.batteryCurrent = sensors[batteryCurrentSensor];
    systemSensorData.batteryLevel = sensors[batteryChargeSensor];
    systemSensorData.batteryTemperature = sensors[batteryTemperatureSensor];
    const short statusValue = static_cast<short>(sensors[batteryStatusSensor]);
    systemSensorData.batteryCharging = statusValue & 0b10000000;
}
