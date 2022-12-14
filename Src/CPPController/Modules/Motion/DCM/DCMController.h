#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/MotionControl/FootTask.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Motion/BalanceTarget.h"
#include "Representations/Motion/CoMProjectionEstimation.h"

#include "Modules/Motion/Utils/FloatingBaseObserver.h"
#include "Modules/Motion/Utils/NetWrenchObserver.h"
#include "Modules/Motion/Utils/Contact.h"
#include "Modules/Motion/Utils/LowPassVelocityFilter.h"
#include "Modules/Motion/Utils/Stabilizer.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/Blackboard.h"

#include <fstream>

class DCMControllerBase
{
public:
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(RobotModel);
    REQUIRES_REPRESENTATION(JointAngles);
    REQUIRES_REPRESENTATION(CoMProjectionEstimation);
    
    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(LeftFootTask);
    USES_REPRESENTATION(RightFootTask);
    USES_REPRESENTATION(NetWrenchEstimation);
    USES_REPRESENTATION(LegMotionSelection);
    USES_REPRESENTATION(StiffnessSettings);
    USES_REPRESENTATION(BalanceActionSelection);

    MODIFIES_REPRESENTATION(FloatingBaseEstimation);
    MODIFIES_REPRESENTATION(BalanceTarget);
};

class DCMController : public DCMControllerBase
{
public:
    DCMController();
    ~DCMController();
    
    void update(DCMJointRequest &s);

private:
    void update();
    void run(DCMJointRequest &s);
    void configureOnce();
    void updateRealFromKinematics();
    Contact supportContact();
    void applyAnkleControl();
    void applyCoMControl();
    void applyCoMControlWithoutDCMFeedback();

private:
    const float dt_ = Constants::motionCycleTime;
    float leftFootRatio_ = 0.5f;
    Vector3f controlCom_;
    Vector3f controlComd_;
    Matrix3f pelvisOrientation_ = Matrix3f::Identity();
    Vector3f realCom_;
    Vector3f realComd_;
    FloatingBaseObserver floatingBaseObs_;
    NetWrenchObserver netWrenchObs_;
    LowPassVelocityFilter<Vector3f> comVelFilter_;
    LowPassVelocityFilter<Vector2f> leftAnkleVelFilter_;
    LowPassVelocityFilter<Vector2f> rightAnkleVelFilter_;
    Stabilizer stabilizer_;

    float leftRollD = 0.f;
    float leftPitchD = 0.f;
    float rightRollD = 0.f;
    float rightPitchD = 0.f;

    bool updateJointRequest = false;

private:
    static constexpr float FOOT_DAMPING_ADMITTANCE_K_TauX = 0.1f;
    static constexpr float FOOT_DAMPING_ADMITTANCE_K_TauY = 0.1f;

    unsigned int dcmFinishedCounter = 0;

    const int hipHeight = MotionConfig::hipHeight;
    JointRequest jointRequest_;

    std::ofstream f_dcm_ecop;
    std::ofstream f_comd;
};