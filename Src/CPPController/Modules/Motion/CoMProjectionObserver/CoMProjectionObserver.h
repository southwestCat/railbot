#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Tools/Module/Blackboard.h"

class CoMProjectionObserverBase
{
public:
    REQUIRES_REPRESENTATION(FsrSensorData);
};

class CoMProjectionObserver : public CoMProjectionObserverBase
{
public:
    void update(CoMProjectionEstimation &o);

private:
    void update();
};
