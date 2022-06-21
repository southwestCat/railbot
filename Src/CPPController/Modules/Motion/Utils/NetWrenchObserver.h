#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "SpaceVecAlg/ForceVec.h"
#include "Modules/Motion/Utils/Contact.h"
#include "Tools/Module/Blackboard.h"

class NetWrenchObserverBase
{
public:
    REQUIRES_REPRESENTATION(FsrFilteredData);
    // REQUIRES_REPRESENTATION(FsrSensorData);
    REQUIRES_REPRESENTATION(RobotModel);

    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(FloatingBaseEstimation);

    MODIFIES_REPRESENTATION(NetWrenchEstimation);
};

class NetWrenchObserver : public NetWrenchObserverBase
{
public:
    void update(const Contact &contact);
    const sva::ForceVec &wrench() const { return netWrench_; }
    const Vector3f &zmp() const { return netZMP_; }

private:
    void update();
    void updateNetWrench();
    void updateZMP(const Contact &contact);
    sva::ForceVec calcWrench(float f, Vector2f p);

private:
    sva::ForceVec netWrench_;
    Vector3f netZMP_;
    // const FsrSensorData *fsr;
};
