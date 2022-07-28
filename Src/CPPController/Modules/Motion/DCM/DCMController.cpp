#include "DCMController.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"

DCMController::DCMController()
{
    f_dcm_ecop.open("dcm_ecop.txt");
    f_comd.open("comd.txt");
}

DCMController::~DCMController()
{
    f_dcm_ecop.close();
    f_comd.close();
}

void DCMController::update()
{
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(RobotModel);
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(CoMProjectionEstimation);

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
    // applyCoMControl();
    applyCoMControlWithoutDCMFeedback();

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
    // realCom_ = theFloatingBaseEstimation->WTB.translation();
    realCom_ = theFloatingBaseEstimation->WTO.translation();
    comVelFilter_.update(realCom_);
    realComd_ = comVelFilter_.vel();
    theFloatingBaseEstimation->comVelocity = realComd_;

    //! LOG
    if (theBalanceTarget->balanceEngineReadyPosture)
    {
        f_comd << "[" << theFrameInfo->time << "]" << " xd: " << realComd_.x() << " yd: " << realComd_.y() << " zd: " << realComd_.z() << std::endl;
    }

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

    const float copX = (leftCoP.x() + rightCoP.x()) / 2.f;
    const float desiredX = 15.f;
    const float Kx = 0.3f;

    float errX = desiredX - copX;
    float comxd = errX * Kx;

    //! Get soleLeft target and soleRight target from BalanceTarget.
    Pose3f targetSoleLeft = theBalanceTarget->soleLeftRequest;
    Pose3f targetSoleRight = theBalanceTarget->soleRightRequest;

    //! Calculate new sole left and right targets
    targetSoleLeft.translation.x() -= comxd * dt_;
    targetSoleRight.translation.x() -= comxd * dt_;

    //! Update jointRequest_
    bool isPossible = InverseKinematic::calcLegJoints(targetSoleLeft, targetSoleRight, Vector2f::Zero(), jointRequest_, *theRobotDimensions);

    updateJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetSoleLeft;
    theBalanceTarget->soleRightRequest = targetSoleRight;

    //! control done flag.
    if (abs(comxd) < 1.f)
        theBalanceTarget->isDCMControlDone = true;
    else
        theBalanceTarget->isDCMControlDone = false;
}

void DCMController::applyCoMControlWithoutDCMFeedback()
{
    const float Kx = 5.f;

    //! Calculate copX error.
    // const float eCoPNX = theCoMProjectionEstimation->estimatedCoPNormalized.x();
    const float eCoPX = theCoMProjectionEstimation->estimatedCoP.x();
    float errX = 0.f - eCoPX;
    float comxd = errX * Kx;
    if (comxd > 15.f)
        comxd = 15.f;
    if (comxd < -15.f)
        comxd = -15.f;

    //! LOG f
    const float ecopNX = theCoMProjectionEstimation->estimatedCoPNormalized.x();
    const float mcopNX = theCoMProjectionEstimation->measuredCoPNormalized.x();
    f_dcm_ecop << "[" << theFrameInfo->time << "]" << " ecopX: " << ecopNX << " mcopX: " << mcopNX << " comdX: " << comxd << std::endl;

    //! Get soleLeft target and soleRight target from BalanceTarget.
    Pose3f targetSoleLeft = theBalanceTarget->soleLeftRequest;
    Pose3f targetSoleRight = theBalanceTarget->soleRightRequest;

    //! Calculate new sole left and right targets
    targetSoleLeft.translation.x() -= comxd * dt_;
    targetSoleRight.translation.x() -= comxd * dt_;

    //! Update jointRequest_
    bool isPossible = InverseKinematic::calcLegJoints(targetSoleLeft, targetSoleRight, Vector2f::Zero(), jointRequest_, *theRobotDimensions);

    updateJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetSoleLeft;
    theBalanceTarget->soleRightRequest = targetSoleRight;

    const float errNX = abs(ecopNX - mcopNX);

    if (errNX < 0.08f)
    {
        // theBalanceTarget->isDCMControlDone = true;
        dcmFinishedCounter++;
    }
    else
    {
        // theBalanceTarget->isDCMControlDone = false;
        dcmFinishedCounter = 0;
    }
    if (dcmFinishedCounter > 50)
        theBalanceTarget->isDCMControlDone = true;
    else
        theBalanceTarget->isDCMControlDone = false;
}
