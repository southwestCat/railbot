#include "LIPMController.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"

void LIPMController::update()
{
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(RobotModel);
    UPDATE_REPRESENTATION(JointAngles);

    // printf("%3.3f\n", (float)toDegrees(theJointAngles->angles[Joints::lHipPitch]));
}

void LIPMController::update(StabilizerJointRequest &s)
{
    update();
    

    if (theLegMotionSelection->targetMotion != MotionRequest::balance)
    {
        startTime = theFrameInfo->time;
        float soleL = theRobotModel->soleLeft.translation.z();
        float soleR = theRobotModel->soleRight.translation.z();
        initHeight = abs((soleL + soleR) / 2.f);

        startJoints_ = *theJointAngles;

        return;
    }

    //! stand high to stand posture.
    if (!readyPosture(s))
        return;

    run(s);
}

bool LIPMController::readyPosture(StabilizerJointRequest &s)
{
    unsigned nowTime = theFrameInfo->time - startTime;
    if (nowTime > readyPostureTime)
    {
        return true;
    }

    float t = (float)nowTime / 1000.f;
    const float duringT = (readyPostureTime) / 1000.f;
    float target = hipHeight;
    Pose3f targetL = Pose3f(Vector3f(0.f, theRobotDimensions->yHipOffset, -target));
    Pose3f targetR = Pose3f(Vector3f(0.f, -theRobotDimensions->yHipOffset, -target));
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), targetJointRequest_, *theRobotDimensions);
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        jointRequest_.angles[i] = startJoints_.angles[i] + t / duringT * (targetJointRequest_.angles[i] - startJoints_.angles[i]);
    }
    if (isPossible)
    {
        MotionUtilities::copy(jointRequest_, s, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
    }
    return false;
}

void LIPMController::configureOnce()
{
    static bool once = true;
    if (once)
    {
        comVelFilter_.cutoffPeriod(dt_ * 2);
        leftAnkleVelFilter_.cutoffPeriod(dt_ * 2);
        rightAnkleVelFilter_.cutoffPeriod(dt_ * 2);
        stabilizer_.reset();
        stabilizer_.configure();
        once = false;
    }
}

void LIPMController::run(StabilizerJointRequest &s)
{
    //! Configure once.
    configureOnce();

    //! Floating base estimation
    floatingBaseObs_.leftFootRatio(leftFootRatio_);
    floatingBaseObs_.run();

    //! Update real com information
    updateRealFromKinematics();

    //! contact frame presented in BH robot frame.
    sva::PTransform X_0_a = floatingBaseObs_.getAnchorFrame();
    pelvisOrientation_ = X_0_a.rotation();
    //! set pelvis orientation
    //! set torso orientation

    //! Stabilizer
    netWrenchObs_.update(supportContact());
    stabilizer_.updateState(realCom_, realComd_, netWrenchObs_.wrench(), leftFootRatio_);
    stabilizer_.run();

    //! Apply control
    applyAnkleControl(s);
    // applyCoMControl(s);
}

void LIPMController::updateRealFromKinematics()
{
    floatingBaseObs_.updateRobot(*theFloatingBaseEstimation);
    realCom_ = theFloatingBaseEstimation->WTB.translation();
    comVelFilter_.update(realCom_);
    realComd_ = comVelFilter_.vel();

    // std::cout << realCom_.transpose() << std::endl;
    // std::cout << theRobotModel->centerOfMass.transpose() << std::endl;
}

Contact LIPMController::supportContact()
{
    sva::PTransform pose = {Matrix3f::Identity(), {0.f, 0.f, 0.f}};
    return Contact(0.f, 0.f, pose, Contact::SurfaceType::defaultContact);
}

void LIPMController::applyAnkleControl(StabilizerJointRequest &s)
{
    float left_roll_d;
    float left_pitch_d;
    float right_roll_d;
    float right_pitch_d;

    //! equation (15) "Stair climbing stabilization of the HRP-4 humanoid robot using whole-bosd admittance control"
    float A_copy = FOOT_DAMPING_ADMITTANCE_K_TauY;
    float A_copx = FOOT_DAMPING_ADMITTANCE_K_TauX;

    Matrix2x3f A_cop;
    A_cop << A_copy, 0, 0, 0, A_copx, 0;

    const Vector2f leftCoP = theLeftFootTask->targetCoP;
    const Vector2f rightCoP = theRightFootTask->targetCoP;

    //! Convert to sole frame
    // Vector3f left_cop_target = Vector3f(leftCoP.x(), leftCoP.y(), 0.f) + Vector3f(30.f, 5.62f, 0.f);
    // Vector3f right_cop_target = Vector3f(rightCoP.x(), rightCoP.y(), 0.f) + Vector3f(30.f, -5.62f, 0.f);
    Vector3f left_cop_target = Vector3f(30.f, 5.62f, 0.f);
    Vector3f right_cop_target = Vector3f(30.f, -5.62f, 0.f);

    // mm to meter
    left_cop_target.x() /= 1000.f;
    left_cop_target.y() /= 1000.f;
    right_cop_target.x() /= 1000.f;
    right_cop_target.y() /= 1000.f;

    const Vector3f &left_f_measure = theNetWrenchEstimation->wrenchLeft.force();
    const Vector3f &left_tau_measure = theNetWrenchEstimation->wrenchLeft.couple();
    const Vector3f &right_f_measure = theNetWrenchEstimation->wrenchRight.force();
    const Vector3f &right_tau_measure = theNetWrenchEstimation->wrenchRight.couple();

    Vector2f left_rollpitch_d = A_cop * (left_cop_target.cross(left_f_measure) - left_tau_measure);
    Vector2f right_rollpitch_d = A_cop * (right_cop_target.cross(right_f_measure) - right_tau_measure);

    leftAnkleVelFilter_.update(left_rollpitch_d);
    rightAnkleVelFilter_.update(right_rollpitch_d);

    left_roll_d = leftAnkleVelFilter_.vel().x();
    left_pitch_d = leftAnkleVelFilter_.vel().y();
    right_roll_d = rightAnkleVelFilter_.vel().x();
    right_pitch_d = rightAnkleVelFilter_.vel().y();

    leftRollD = left_roll_d;
    leftPitchD = left_pitch_d;
    rightRollD = right_roll_d;
    rightPitchD = right_pitch_d;

    // static float timeNow = 0.f;
    // float angleOffset = cos(pi/2.f * timeNow) * 3_deg;
    // timeNow += dt_;

    // jointRequest.angles[Joints::lAnklePitch] += angleOffset * dt_;
    // jointRequest.angles[Joints::rAnklePitch] += angleOffset * dt_;

    // jointRequest.angles[Joints::lAnkleRoll] += left_roll_d * dt_;
    // s.angles[Joints::lAnklePitch] += left_pitch_d * dt_;
    // jointRequest.angles[Joints::rAnkleRoll] += right_roll_d * dt_;
    // s.angles[Joints::rAnklePitch] += right_pitch_d * dt_;

    // printf(">\n");
    // printf("left: %3.3f right: %3.3f\n", left_pitch_d, right_pitch_d);
    // printf("----\n\n");

    // printf(">\n");
    // printf("left:\n");
    // printf("roll pitch: %3.3f %3.3f\n", left_roll_d, left_pitch_d);
    // printf("right:\n");
    // printf("roll pitch: %3.3f %3.3f\n", right_roll_d, right_pitch_d);
    // printf(" left cop target: %3.3f %3.3f %3.3f\n", left_cop_target.x(), left_cop_target.y(), left_cop_target.z());
    // printf("right cop target: %3.3f %3.3f %3.3f\n", right_cop_target.x(), right_cop_target.y(), right_cop_target.z());
    // printf("      left force: %3.3f %3.3f %3.3f\n", left_f_measure.x(), left_f_measure.y(), left_f_measure.z());
    // printf("        left tau: %3.3f %3.3f %3.3f\n", left_tau_measure.x(), left_tau_measure.y(), left_tau_measure.z());
    // printf("     right force: %3.3f %3.3f %3.3f\n", right_f_measure.x(), right_f_measure.y(), right_f_measure.z());
    // printf("       right tau: %3.3f %3.3f %3.3f\n", right_tau_measure.x(), right_tau_measure.y(), right_tau_measure.z());
    // std::cout << jointRequest.angles[Joints::lAnklePitch] << std::endl;
    // printf("----\n\n");
}

void LIPMController::applyCoMControl(StabilizerJointRequest &s)
{
    const sva::PTransform &WTB = theFloatingBaseEstimation->WTB;
    const sva::PTransform &OTA = theFloatingBaseEstimation->OTA;

    const Vector3f &com = WTB.translation();
    Vector3f origin = OTA.inv().translation();
    float comx = com.x();

    float comK = 0.5f;
    static float timeNow = 0.f;
    // float comx_desire = 5.f * sin(pi / 4.f * timeNow - pi / 2.f) + 5.f;
    float comx_desire = 0.f;
    timeNow += dt_;
    float comx_err = comx_desire - comx;

    if (abs(comx_err) < 2.f)
    {
        comK = 0.5f;
    }
    else if (abs(comx_err) < 5.f)
    {
        comK = 1.f;
    }
    else
    {
        comK = 2.f;
    }
    float comxd = comx_err * comK;
    origin.x() += comxd * dt_;
    // origin.x() = 10.f;
    origin.z() = MotionConfig::hipHeight;

    sva::PTransform modified_ATO = {Matrix3f::Identity(), origin};
    Vector3f p_soleLeft = modified_ATO.inv().translation() + Vector3f(0.f, 50.f, 0.f);
    Vector3f p_soleRight = modified_ATO.inv().translation() + Vector3f(0.f, -50.f, 0.f);

    Pose3f targetL(p_soleLeft);
    Pose3f targetR(p_soleRight);

    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
}
