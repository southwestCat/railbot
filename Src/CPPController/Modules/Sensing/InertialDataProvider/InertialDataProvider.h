#pragma once

#include "Representations/Sensing/InertialData.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Tools/Module/Blackboard.h"

class InertialDataProviderBase
{
public:
    REQUIRES_REPRESENTATION(InertialSensorData);
    REQUIRES_REPRESENTATION(IMUCalibration);
};

class InertialDataProvider : public InertialDataProviderBase
{
public:
    InertialDataProvider();
    void update(InertialData &inertialData);

private:
    float beta;
    float q0, q1, q2, q3;

    void update();
};