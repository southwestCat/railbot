#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/MotionControl/FootTask.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Modules/Motion/Utils/FloatingBaseObserver.h"
#include "Modules/Motion/Utils/NetWrenchObserver.h"
#include "Modules/Motion/Utils/Contact.h"
#include "Modules/Motion/Utils/LowPassVelocityFilter.h"
#include "Modules/Motion/Utils/Stabilizer.h"
#include "Tools/Module/Blackboard.h"

class LIPMControllerBase
{
public:
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(RobotDimensions);
    REQUIRES_REPRESENTATION(FrameInfo);
    
    USES_REPRESENTATION(LeftFootTask);
    USES_REPRESENTATION(RightFootTask);
    USES_REPRESENTATION(NetWrenchEstimation);

    MODIFIES_REPRESENTATION(FloatingBaseEstimation);
};

class LIPMController : public LIPMControllerBase
{
public:
    void update(StabilizerJointRequest &s);

private:
    void update();
    void run();
    void configureOnce();
    void updateRealFromKinematics();
    Contact supportContact();
    void applyAnkleControl();
    void applyCoMControl();

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

private:
    static constexpr float FOOT_DAMPING_ADMITTANCE_K_TauX = 0.001f;
    static constexpr float FOOT_DAMPING_ADMITTANCE_K_TauY = 0.f;
};