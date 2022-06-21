#include "DCMController.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"

void DCMController::update()
{
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(RobotModel);
    UPDATE_REPRESENTATION(JointAngles);

    // printf("%3.3f\n", (float)toDegrees(theJointAngles->angles[Joints::lHipPitch]));
}

void DCMController::update(DCMJointRequest &s)
{
    update();

    //! Update FloatingBaseEstimation and NetWrenchEstimation
    run(s);
}

void DCMController::configureOnce()
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

void DCMController::run(DCMJointRequest &s)
{
    //! Configure once.
    configureOnce();

    //! Floating base estimation
    floatingBaseObs_.leftFootRatio(leftFootRatio_);
    floatingBaseObs_.run();

    //! Update real com information
    updateRealFromKinematics();

    //! contact frame presented in BH robot frame.
    // sva::PTransform X_0_a = floatingBaseObs_.getAnchorFrame();
    // pelvisOrientation_ = X_0_a.rotation();
    //! set pelvis orientation
    //! set torso orientation

    //! Stabilizer
    netWrenchObs_.update(supportContact());

    //! QP stabilizer
    stabilizer_.updateState(realCom_, realComd_, netWrenchObs_.wrench(), leftFootRatio_);
    stabilizer_.run();

    //! Only run at BalanceActionSelection::dcm
    if (theBalanceActionSelection->targetAction != BalanceActionSelection::dcm)
        return;

    //! Initial JointRequest
    for (int i = 0; i <= Joints::rAnkleRoll; i++)
    {
        s.angles[i] = theBalanceTarget->lastJointRequest.angles[i];
    }

    //! Apply control
    applyCoMControl();

    //! Apply JointRequest
    if (updateJointRequest)
    {
        for (unsigned i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            s.angles[i] = jointRequest_.angles[i];
        }
    }
}

void DCMController::updateRealFromKinematics()
{
    floatingBaseObs_.updateRobot(*theFloatingBaseEstimation);
    realCom_ = theFloatingBaseEstimation->WTB.translation();
    comVelFilter_.update(realCom_);
    realComd_ = comVelFilter_.vel();
    theFloatingBaseEstimation->comVelocity = realComd_;

    // std::cout << realCom_.transpose() << std::endl;
    // printf("vel: %3.3f, %3.3f\n", realCom_.x(), realCom_.y());
    // std::cout << theRobotModel->centerOfMass.transpose() << std::endl;
}

Contact DCMController::supportContact()
{
    sva::PTransform pose = {Matrix3f::Identity(), {0.f, 0.f, 0.f}};
    return Contact(0.f, 0.f, pose, Contact::SurfaceType::defaultContact);
}

//! Not Recommand
void DCMController::applyAnkleControl()
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

    // std::cout << ">\n";
    // std::cout << " left cop: " << leftCoP.transpose() << std::endl;
    // std::cout << "right cop: " << rightCoP.transpose() << std::endl;
    // std::cout << "----\n\n";
    //! Convert to sole frame
    Vector3f left_cop_target = Vector3f(leftCoP.x(), leftCoP.y(), 0.f) + Vector3f(30.f, 5.62f, 0.f);
    Vector3f right_cop_target = Vector3f(rightCoP.x(), rightCoP.y(), 0.f) + Vector3f(30.f, -5.62f, 0.f);

    // Vector3f left_cop_target = Vector3f(30.f, 5.62f, 0.f);
    // Vector3f right_cop_target = Vector3f(30.f, -5.62f, 0.f);

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

    jointRequest_.angles[Joints::lAnklePitch] += leftPitchD * dt_;
    jointRequest_.angles[Joints::rAnklePitch] += rightPitchD * dt_;

    //! Update BalanceTareger
    theBalanceTarget->lastJointRequest = jointRequest_;

    // static float timeNow = 0.f;
    // float angleOffset = cos(pi/2.f * timeNow) * 3_deg;
    // timeNow += dt_;
    // s.angles[Joints::lAnklePitch] += angleOffset * dt_;
    // s.angles[Joints::rAnklePitch] += angleOffset * dt_;

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

void DCMController::applyCoMControl()
{
    const Vector2f leftCoP = theLeftFootTask->targetCoP;
    const Vector2f rightCoP = theRightFootTask->targetCoP;

    Vector3f left_cop_target = Vector3f(leftCoP.x(), leftCoP.y(), 0.f) + Vector3f(30.f, 5.62f, 0.f);
    Vector3f right_cop_target = Vector3f(rightCoP.x(), rightCoP.y(), 0.f) + Vector3f(30.f, -5.62f, 0.f);

    // mm to meter
    left_cop_target.x() /= 1000.f;
    left_cop_target.y() /= 1000.f;
    right_cop_target.x() /= 1000.f;
    right_cop_target.y() /= 1000.f;

    const Vector3f &left_f_measure = theNetWrenchEstimation->wrenchLeft.force();
    const Vector3f &left_tau_measure = theNetWrenchEstimation->wrenchLeft.couple();
    const Vector3f &right_f_measure = theNetWrenchEstimation->wrenchRight.force();
    const Vector3f &right_tau_measure = theNetWrenchEstimation->wrenchRight.couple();

    const float A_copy = 1000.f;
    const float A_copx = 1000.f;

    Matrix2x3f A_cop;
    A_cop << A_copy, 0, 0, 0, A_copx, 0;

    Vector2f left_rollpitch_d = A_cop * (left_cop_target.cross(left_f_measure) - left_tau_measure);
    Vector2f right_rollpitch_d = A_cop * (right_cop_target.cross(right_f_measure) - right_tau_measure);

    leftAnkleVelFilter_.update(left_rollpitch_d);
    rightAnkleVelFilter_.update(right_rollpitch_d);

    const float left_pitch_d = leftAnkleVelFilter_.vel().y();
    const float right_pitch_d = rightAnkleVelFilter_.vel().y();

    const float pitchD = (left_pitch_d + right_pitch_d) / 2.f;

    // printf(">\n");
    // printf("left: %3.3f right: %3.3f\n", left_pitch_d, right_pitch_d);
    // printf("----\n\n");

    // const sva::PTransform &WTB = theFloatingBaseEstimation->WTB;
    // const sva::PTransform &OTA = theFloatingBaseEstimation->OTA;

    // const Vector3f &com = WTB.translation();
    // Vector3f origin = OTA.inv().translation();
    // float comx = com.x();

    // float comK = 0.3f;
    // float comx_desire = 0.f;
    // float comx_err = comx_desire - comx;

    // float comxd = comx_err * comK;
    // origin.x() += comxd * dt_;
    // // origin.x() = 10.f;
    // origin.z() = MotionConfig::hipHeight;

    // sva::PTransform modified_ATO = {Matrix3f::Identity(), origin};
    // Vector3f p_soleLeft = modified_ATO.inv().translation() + Vector3f(0.f, 50.f, 0.f);
    // Vector3f p_soleRight = modified_ATO.inv().translation() + Vector3f(0.f, -50.f, 0.f);

    // Pose3f targetL(p_soleLeft);
    // Pose3f targetR(p_soleRight);

    // bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    // updateJointRequest = isPossible;

    // //! Update BalanceTarget
    // theBalanceTarget->lastJointRequest = jointRequest_;
    // theBalanceTarget->soleLeftRequest = targetL;
    // theBalanceTarget->soleRightRequest = targetR;
}
