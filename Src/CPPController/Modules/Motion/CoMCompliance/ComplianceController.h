#pragma once

#include "Representations/Motion/CoMProjectionEstimation.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Module/Blackboard.h"

class ComplianceControllerBase
{
public:
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(BalanceActionSelection);
    REQUIRES_REPRESENTATION(JointAngles);
    
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(NetWrenchEstimation);
    USES_REPRESENTATION(FloatingBaseEstimation);
};

class ComplianceController : public ComplianceControllerBase
{
public:
    ComplianceController();
    void update(ComplianceJointRequest &o);

private:
    void update();
    bool inEstimatedEllipseArea(float estimated, float measured, float cov);

private:
    float covRateThreshold; //! base threshold
    float errCOV; //! base covariance of estimated cop 
    float T; //! cutoff time.
    bool keepJoints;
    float Acopx;
    float Acopy;
};