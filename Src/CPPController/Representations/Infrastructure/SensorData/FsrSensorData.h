#pragma once

#include "Tools/Motion/SensorData.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Legs.h"

#include <array>

class FsrSensorData
{
public:
    //! [N]
    std::array<std::array<float, FsrSensors::numOfFsrSensors>, Legs::numOfLegs> pressures;
    std::array<float, Legs::numOfLegs> totals;
};

class FsrFilteredData : public FsrSensorData
{
public:
};