#pragma once

#include "Representations/Sensing/InertialData.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"

class InertialDataProviderBase
{
public:
    REQUIRES_REPRESENTATION(InertialSensorData);
    REQUIRES_REPRESENTATION(IMUCalibration);
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(KeyStates);
};

class InertialDataProvider : public InertialDataProviderBase
{
public:
    InertialDataProvider();
    void update(InertialData &inertialData);

private:
    void update();

private:
    float beta;
    float q0, q1, q2, q3;

    unsigned lastKeyPressed = 0;
};