#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Motion/SensorData.h"

class JointSensorData : public JointAngles
{
public:
    JointSensorData()
    {
        currents.fill(SensorData::off);
        temperatures.fill(0);
        status.fill(regular);
    }

    enum TemperatureStatus
    {
        regular,
        hot,
        veryHot,
        criticallyHot,
        numOfTemperatureStatus
    };
    std::array<short, Joints::numOfJoints> currents;                            /**< The currents of all motors. */
    std::array<unsigned char, Joints::numOfJoints> temperatures;                /**< The currents of all motors. */
    std::array<JointSensorData::TemperatureStatus, Joints::numOfJoints> status; /**< The status of all motors. */
};