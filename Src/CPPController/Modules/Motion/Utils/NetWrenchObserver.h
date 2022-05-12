#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "SpaceVecAlg/ForceVec.h"
#include "Modules/Motion/Utils/Contact.h"
#include "Tools/Module/Blackboard.h"

class NetWrenchObserverBase
{
public:
    REQUIRES_REPRESENTATION(Contact);
};

class NetWrenchObserver
{
public:
    void update(NetWrenchEstimation &n);
    const sva::ForceVec &wrench() const { return netWrench_; }

private:
    void update();
    sva::ForceVec netWrench_;
    Vector3f netZMP_;
    const FsrSensorData *fsr;
};
