#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Tools/Module/Blackboard.h"
#include "Modules/Motion/Utils/ExponentialMovingAverage.h"

class CoMProjectionObserverBase
{
public:
    REQUIRES_REPRESENTATION(RobotModel);

    USES_REPRESENTATION(FloatingBaseEstimation);
    USES_REPRESENTATION(NetWrenchEstimation);
};

class CoMProjectionObserver : public CoMProjectionObserverBase
{
public:
    CoMProjectionObserver();
    void update(CoMProjectionEstimation &o);

private:
    void update();

private:
    ExponentialMovingAverage copFilter;
    Vector2f baseBias; //< normalized bias between estimated and measured.
};
