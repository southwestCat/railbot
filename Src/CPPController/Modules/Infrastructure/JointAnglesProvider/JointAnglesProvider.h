#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Module/Blackboard.h"

class JointAnglesProviderBase
{
public:
    REQUIRES_REPRESENTATION(JointSensorData);
};

class JointAnglesProvider : public JointAnglesProviderBase
{
public:
    void update(JointAngles &jointAngles);
private:
    void update();
};