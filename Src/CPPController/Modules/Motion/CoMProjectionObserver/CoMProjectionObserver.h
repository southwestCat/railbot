#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Tools/Module/Blackboard.h"

class CoMProjectionObserverBase
{
public:
    REQUIRES_REPRESENTATION(FsrSensorData);
    REQUIRES_REPRESENTATION(RobotModel);

    USES_REPRESENTATION(FloatingBaseEstimation);
    USES_REPRESENTATION(NetWrenchEstimation);
};

class CoMProjectionObserver : public CoMProjectionObserverBase
{
public:
    void update(CoMProjectionEstimation &o);

private:
    void update();
};
