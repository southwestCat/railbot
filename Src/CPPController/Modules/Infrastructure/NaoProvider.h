#pragma once

#include "Platform/NaoBody.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/Module/Blackboard.h"

class NaoProviderBase
{
public:
    USES_REPRESENTATION(JointRequest);
    REQUIRES_REPRESENTATION(FrameInfo);
};

class NaoProvider : public NaoProviderBase
{
public:
    NaoProvider();
    ~NaoProvider();

    static thread_local NaoProvider *theInstance; /**< The only instance of this module. */
    NaoBody naoBody;

    static void finishFrame();
    static void waitForFrameData();
    static void setInstance(NaoProvider *instance);
    static NaoProvider &getInstance();

    void update(FrameInfo &frameInfo);
    void update(FsrSensorData &fsrSensorData);
    void update(InertialSensorData &inertialSensorData);
    void update(JointSensorData &jointSensorData);
    void update(KeyStates &keyStates);

private:
    void update();
    void send();
};