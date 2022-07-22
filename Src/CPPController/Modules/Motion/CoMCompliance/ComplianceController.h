#pragma once

#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Motion/BalanceTarget.h"
#include "Tools/Module/Blackboard.h"

#include <fstream>

class ComplianceControllerBase
{
public:
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(BalanceActionSelection);
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(RobotModel);
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(NetWrenchEstimation);
    USES_REPRESENTATION(FloatingBaseEstimation);

    MODIFIES_REPRESENTATION(BalanceTarget);
};

class ComplianceController : public ComplianceControllerBase
{
public:
    ComplianceController();
    ~ComplianceController();
    void update(ComplianceJointRequest &o);

private:
    void update();
    bool inEstimatedEllipseArea(float estimated, float measured, float cov);

private:
    float covRateThreshold; //! base threshold
    float errCOV;           //! base covariance of estimated cop
    float T;                //! cutoff time.
    bool keepJoints;
    float Acopx;
    float Acopy;

    const float dt = Constants::motionCycleTime;

    std::ofstream f_compliance_ecop;
};