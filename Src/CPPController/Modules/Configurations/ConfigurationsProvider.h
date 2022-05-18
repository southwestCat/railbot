#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Tools/Math/Eigen.h"

class ConfigurationsProvider
{
public:
    void update(IMUCalibration &imu);
};
