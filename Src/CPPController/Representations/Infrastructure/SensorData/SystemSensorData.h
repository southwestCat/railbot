#pragma once

#include "Tools/Motion/SensorData.h"

class SystemSensorData
{
public:
    float cpuTemperature = SensorData::off;     /** The temperature of the cpu (in °C). */
    float batteryCurrent = SensorData::off;     /** The current of the battery (in A). */
    float batteryLevel = SensorData::off;       /** The current of the battery (in %). Range: [0.0, 1.0] */
    float batteryTemperature = SensorData::off; /** The temperature of the battery (in %, whatever that means...). Range: [0.0, 1.0] */
    bool batteryCharging = false;               /** The battery is charging */
};