#include "Stabilizer.h"
#include "Modules/Motion/Utils/clamp.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/ModuleManager.h"

Stabilizer::Stabilizer()
    : dcmIntegrator_(5.f), dcmDerivator_(1.f)
{
    log.open("log.txt");
}
Stabilizer::~Stabilizer()
{
    log.close();
}

void Stabilizer::update()
{
    UPDATE_REPRESENTATION(RobotModel);
    // UPDATE_REPRESENTATION(FsrSensorData);
    UPDATE_REPRESENTATION(FsrFilteredData);
    UPDATE_REPRESENTATION(FrameInfo);
}

void Stabilizer::disable()
{
    comAdmittance_.setZero();
    copAdmittance_.setZero();
    dcmGain_ = 0.f;
    dcmIntegralGain_ = 0.f;
    dfzAdmittance_ = 0.f;
    vdcFrequency_ = 0.f;
    vdcStiffness_ = 0.f;
}

void Stabilizer::configure()
{
    reconfigure();
}

void Stabilizer::reconfigure()
{
    fdqpWeights_.netWrenchSqrt = 100.f;
    fdqpWeights_.ankleTorqueSqrt = 10.f;
    fdqpWeights_.forceRatioSqrt = 1.f;

    comAdmittance_ = {0.f, 0.f, 0.007f};
    copAdmittance_ = {0.01f, 0.01f};
    dfzAdmittance_ = 0.0001f;

    dcmGain_ = 1.4f;
    dcmDerivGain_ = 0.f;
    dcmIntegralGain_ = 10.f;
    dcmPropGain_ = 5.f;
    dcmIntegrator_.timeConstant(10.f);
    dcmDerivator_.timeConstant(1.f);

    comStiffness_ = {1000.f, 1000.f, 100.f};
    comWeight_ = 1000.f;

    float d = 300.f;
    float k = 1.f;
    contactDamping_ = sva::MotionVec({d, d, d}, {d, d, d});
    contactStiffness_ = sva::MotionVec({k, k, k}, {k, k, k});
    swingFootStiffness_ = 2000.f;
    swingFootWeight_ = 500.f;

    vdcDamping_ = 0.f;
    vdcFrequency_ = 1.f;
    vdcStiffness_ = 1000.f;

    altccIntegrator_.rate(0.1f);
    zmpccIntegrator_.rate(0.1f);
}

void Stabilizer::reset()
{
    dcmIntegrator_.setZero();
    dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);
    altccIntegrator_.setZero();
    altccIntegrator_.saturation(MAX_ALTCC_COM_OFFSET);
    zmpccIntegrator_.setZero();
    zmpccIntegrator_.saturation(MAX_ZMPCC_COM_OFFSET);

    Vector3f staticForce = -mass_ / 1000.f * Constants::gravity / 1000.f;

    altccCoMAccel_.setZero();
    altccCoMOffset_.setZero();
    altccCoMVel_.setZero();
    altccError_.setZero();
    comOffset_.setZero();
    dcmAverageError_.setZero();
    dcmError_.setZero();
    distribWrench_ = {pendulum_.com().cross(staticForce) / 1000.f, staticForce};
    logMeasuredDFz_ = 0.;
    logMeasuredSTz_ = 0.;
    logTargetDFz_ = 0.;
    logTargetSTz_ = 0.;
    zmpccCoMAccel_.setZero();
    zmpccCoMOffset_.setZero();
    zmpccCoMVel_.setZero();
    zmpccError_.setZero();
}

void Stabilizer::checkGains()
{
    clampInPlace(comAdmittance_.x(), 0., MAX_COM_XY_ADMITTANCE, "CoM x-admittance");
    clampInPlace(comAdmittance_.y(), 0., MAX_COM_XY_ADMITTANCE, "CoM y-admittance");
    clampInPlace(comAdmittance_.z(), 0., MAX_COM_Z_ADMITTANCE, "CoM z-admittance");
    clampInPlace(copAdmittance_.x(), 0., MAX_COP_ADMITTANCE, "CoP x-admittance");
    clampInPlace(copAdmittance_.y(), 0., MAX_COP_ADMITTANCE, "CoP y-admittance");
    clampInPlace(dcmGain_, MIN_DCM_P_GAIN, MAX_DCM_P_GAIN, "DCM x-gain");
    clampInPlace(dcmDerivGain_, 0., MAX_DCM_D_GAIN, "DCM deriv x-gain");
    clampInPlace(dcmIntegralGain_, 0., MAX_DCM_I_GAIN, "DCM integral x-gain");
    clampInPlace(dfzAdmittance_, 0., MAX_DFZ_ADMITTANCE, "DFz admittance");
}

void Stabilizer::updateSupportFootGains()
{
}

void Stabilizer::checkInTheAir()
{
}

void Stabilizer::updateZMPFrame()
{
    const Pose3f soleL = theRobotModel->soleLeft;
    const Pose3f soleR = theRobotModel->soleRight;
    const sva::PTransform X_0_lc(soleL.rotation, soleL.translation);
    const sva::PTransform X_0_rc(soleR.rotation, soleR.translation);

    if (contactState_ == Contact::ContactState::DoubleSupport)
    {
        zmpFrame_ = sva::interpolate(X_0_lc, X_0_rc, 0.5f);
        float xmin = std::min(leftFootContact.xmin(), rightFootContact.xmin());
        float xmax = std::max(leftFootContact.xmax(), rightFootContact.xmax());
        float ymin = std::min(leftFootContact.ymin(), rightFootContact.ymin());
        float ymax = std::max(leftFootContact.ymax(), rightFootContact.ymax());

        Matrix4x2f hrepMat;
        Vector4f hrepVec;
        hrepMat << +1, 0,
            -1, 0,
            0, +1,
            0, -1;
        hrepVec << xmax - zmpFrame_.translation().x(),
            zmpFrame_.translation().x() - xmin,
            ymax - zmpFrame_.translation().y(),
            zmpFrame_.translation().y() - ymin;
        zmpArea_ = HrepXf(hrepMat, hrepVec);
        zmpPolygon_.clear();
        zmpPolygon_.push_back(Vector3f(xmax, ymax, zmpFrame_.translation().z()));
        zmpPolygon_.push_back(Vector3f(xmax, ymin, zmpFrame_.translation().z()));
        zmpPolygon_.push_back(Vector3f(xmin, ymin, zmpFrame_.translation().z()));
        zmpPolygon_.push_back(Vector3f(xmin, ymax, zmpFrame_.translation().z()));
    }
    else if (contactState_ == Contact::ContactState::LeftSupport)
    {
        zmpFrame_ = X_0_lc;
        zmpArea_ = leftFootContact.localHrep();
        zmpPolygon_.clear();
        zmpPolygon_.push_back(leftFootContact.vertex0());
        zmpPolygon_.push_back(leftFootContact.vertex1());
        zmpPolygon_.push_back(leftFootContact.vertex2());
        zmpPolygon_.push_back(leftFootContact.vertex3());
    }
    else
    {
        zmpFrame_ = X_0_rc;
        zmpArea_ = rightFootContact.localHrep();
        zmpPolygon_.clear();
        zmpPolygon_.push_back(rightFootContact.vertex0());
        zmpPolygon_.push_back(rightFootContact.vertex1());
        zmpPolygon_.push_back(rightFootContact.vertex2());
        zmpPolygon_.push_back(rightFootContact.vertex3());
    }
    // measuredZMP_ = computeZMP();
    measuredZMP_ = theNetWrenchEstimation->netZMP;
    
    // float zmpx = measuredZMP_.x();
    // float zmpy = measuredZMP_.y();
    // float zmpz = measuredZMP_.z();
    // printf(">\n");
    // printf("zmp: %3.3f %3.3f %3.3f\n", zmpx, zmpy, zmpz);
    // printf("----\n\n");
}

Vector3f Stabilizer::computeZMP() const
{
    const Pose3f &soleL = theRobotModel->soleLeft;
    const Pose3f &soleR = theRobotModel->soleRight;
    // const FsrSensorData &fsr = *theFsrSensorData;
    const FsrSensorData &fsr = *theFsrFilteredData;

    //! Force
    const float &Flfl = fsr.pressures[Legs::left][FsrSensors::fl];
    const float &Flfr = fsr.pressures[Legs::left][FsrSensors::fr];
    const float &Flbl = fsr.pressures[Legs::left][FsrSensors::bl];
    const float &Flbr = fsr.pressures[Legs::left][FsrSensors::br];

    const float &Frfl = fsr.pressures[Legs::right][FsrSensors::fl];
    const float &Frfr = fsr.pressures[Legs::right][FsrSensors::fr];
    const float &Frbl = fsr.pressures[Legs::right][FsrSensors::bl];
    const float &Frbr = fsr.pressures[Legs::right][FsrSensors::br];

    //! Position
    const Vector2f &Plfl = theRobotDimensions->leftFsrPositions[FsrSensors::fl];
    const Vector2f &Plfr = theRobotDimensions->leftFsrPositions[FsrSensors::fr];
    const Vector2f &Plbl = theRobotDimensions->leftFsrPositions[FsrSensors::bl];
    const Vector2f &Plbr = theRobotDimensions->leftFsrPositions[FsrSensors::br];

    const Vector2f &Prfl = theRobotDimensions->rightFsrPositions[FsrSensors::fl];
    const Vector2f &Prfr = theRobotDimensions->rightFsrPositions[FsrSensors::fr];
    const Vector2f &Prbl = theRobotDimensions->rightFsrPositions[FsrSensors::bl];
    const Vector2f &Prbr = theRobotDimensions->rightFsrPositions[FsrSensors::br];

    float Fleft = Flfl + Flfr + Flbl + Flbr;
    float Fright = Frfl + Frfr + Frbl + Frbr;

    if (Fleft < 10 || Fright < 10)
    {
        return Vector3f(-1.f, -1.f, -1.f);
    }

    float lpx = (Flfl * Plfl.x() + Flfr * Plfr.x() + Flbl * Plbl.x() + Flbr * Plbr.x()) / Fleft;
    float lpy = (Flfl * Plfl.y() + Flfr * Plfr.y() + Flbl * Plbl.y() + Flbr * Plbr.y()) / Fleft;
    Vector3f lzmp = (soleL + Vector3f(lpx, lpy, 0.f)).translation;

    float rpx = (Frfl * Prfl.x() + Frfr * Prfr.x() + Frbl * Prbl.x() + Frbr * Prbr.x()) / Fright;
    float rpy = (Frfl * Prfl.y() + Frfr * Prfr.y() + Frbl * Prbl.y() + Frbr * Prbr.y()) / Fright;
    Vector3f rzmp = (soleR + Vector3f(rpx, rpy, 0.f)).translation;
    return (lzmp + rzmp) / 2.f;
}

Vector3f Stabilizer::computeZMP(const sva::ForceVec &wrench) const
{
    Vector3f n = zmpFrame_.rotation().row(2);
    Vector3f p = zmpFrame_.translation();
    const Vector3f &force = wrench.force();
    float normalForce = n.dot(force);
    if (normalForce < 1.f)
    {
        float lambda = pendulum_.omega() * pendulum_.omega();
        return measuredCoM_ + Constants::gravity / lambda;
    }
    const Vector3f &moment_0 = wrench.couple();
    Vector3f moment_p = moment_0 - p.cross(force);
    return p + n.cross(moment_p) / normalForce;
}

void Stabilizer::run()
{
    update();

    resetPendulum();
    updateMass(theMassCalibration->totalMass);
    configureContact();

    checkGains();
    checkInTheAir(); // void
    updateSupportFootGains();
    updateZMPFrame();

    sva::ForceVec desiredWrench = computeDesiredWrench();

    distributeWrench(desiredWrench);
    updateCoMTaskZMPCC();
    updateFootForceDifferenceControl();
}

sva::ForceVec Stabilizer::computeDesiredWrench()
{
    sva::ForceVec desiredWrench;
    if (model_ == TemplateModel::LinearInvertedPendulum)
    {
        desiredWrench = computeLIPDesiredWrench();
    }
    else
    {
        desiredWrench = computeVHIPDesiredWrench();
    }
    return desiredWrench;
}

sva::ForceVec Stabilizer::computeLIPDesiredWrench()
{
    float omega = pendulum_.omega();
    Vector3f comError = pendulum_.com() - measuredCoM_;
    Vector3f comdError = pendulum_.comd() - measuredCoMd_;

    dcmError_ = comError + comdError / omega;
    dcmError_.z() = 0.f;

    zmpError_ = pendulum_.zmp() - measuredZMP_;

    zmpError_.z() = 0.f;
    dcmDerivator_.update(omega * (dcmError_ - zmpError_));
    dcmIntegrator_.append(dcmError_);

    dcmAverageError_ = dcmIntegrator_.eval();
    dcmVelError_ = dcmDerivator_.eval();

    Vector3f desiredCoMAccel = pendulum_.comdd();
    // desiredCoMAccel += omega * (dcmPropGain_ * dcmError_ + comdError);
    // desiredCoMAccel += omega * dcmIntegralGain_ * dcmAverageError_;
    // desiredCoMAccel += omega * dcmDerivGain_ * dcmVelError_;

    Vector3f desiredForce = mass_ / 1000.f * (desiredCoMAccel - Constants::gravity) / 1000.f;

    std::cout << mass_ / 1000.f * Constants::g_1000 << std::endl;

    return {measuredCoM_.cross(desiredForce) / 1000.f, desiredForce};
}

sva::ForceVec Stabilizer::computeVHIPDesiredWrench()
{
    // float vrpGain = dcmGain_ + 1.f;
    // float refOmega = pendulum_.omega();
    // float refLambda = refOmega * refOmega;
    // Vector3f comError = measuredCoM_ - pendulum_.com();

    return sva::ForceVec();
}

void Stabilizer::distributeWrench(const sva::ForceVec &desiredWrench)
{
    const RobotModel &model = *theRobotModel;
    const sva::PTransform &WTO = theFloatingBaseEstimation->WTO;
    //! World frame in left contact frame.
    const sva::PTransform &X_lc_0 = leftFootContact.poseW().inv().toMeter();
    //! World frame in right contact frame.
    const sva::PTransform &X_rc_0 = rightFootContact.poseW().inv().toMeter();
    //! World frame in left ankle frame.
    const sva::PTransform &X_lankle_0 = leftFootContact.anklePose(model, WTO).inv().toMeter();
    //! World frame in right ankle frame.
    const sva::PTransform &X_rankle_0 = rightFootContact.anklePose(model, WTO).inv().toMeter();

    constexpr unsigned NB_VAR = 6 + 6;
    constexpr unsigned COST_DIM = 6 + NB_VAR + 1;
    MatrixXd A;
    VectorXd b;
    A.setZero(COST_DIM, NB_VAR);
    b.setZero(COST_DIM);

    // |w_l_0 + w_r_0 - desiredWrench|^2
    auto A_net = A.block<6, 12>(0, 0);
    auto b_net = b.segment<6>(0);
    A_net.block<6, 6>(0, 0) = Matrix6d::Identity();
    A_net.block<6, 6>(0, 6) = Matrix6d::Identity();
    b_net = desiredWrench.vector().cast<double>();

    // |ankle torques|^2
    auto A_lankle = A.block<6, 6>(6, 0);
    auto A_rankle = A.block<6, 6>(12, 6);
    // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
    A_lankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
    A_rankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;

    A_lankle *= X_lankle_0.dualMatrix().cast<double>();
    A_rankle *= X_rankle_0.dualMatrix().cast<double>();

    // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
    double lfr = leftFootRatio_;
    auto A_fratio = A.block<1, 12>(18, 0);
    A_fratio.block<1, 6>(0, 0) = (1 - lfr) * X_lc_0.dualMatrix().cast<double>().bottomRows<1>();
    A_fratio.block<1, 6>(0, 6) = -lfr * X_rc_0.dualMatrix().cast<double>().bottomRows<1>();

    // Apply weights
    A_net *= (double)fdqpWeights_.netWrenchSqrt;
    b_net *= (double)fdqpWeights_.netWrenchSqrt;
    A_lankle *= (double)fdqpWeights_.ankleTorqueSqrt;
    A_rankle *= (double)fdqpWeights_.ankleTorqueSqrt;

    A_fratio *= (double)fdqpWeights_.forceRatioSqrt;

    MatrixXd Q = A.transpose() * A;
    VectorXd c = -A.transpose() * b;

    constexpr unsigned NB_CONS = 16 + 16 + 2;
    Eigen::Matrix<double, NB_CONS, NB_VAR> A_ineq;
    Eigen::VectorXd b_ineq;
    A_ineq.setZero(NB_CONS, NB_VAR);
    b_ineq.setZero(NB_CONS);
    // CWC * w_l_lc <= 0
    A_ineq.block<16, 6>(0, 0) = wrenchFaceMatrix_.cast<double>() * X_lc_0.dualMatrix().cast<double>();
    // b_ineq.segment<16>(0) is already zero
    // CWC * w_r_rc <= 0
    A_ineq.block<16, 6>(16, 6) = wrenchFaceMatrix_.cast<double>() * X_rc_0.dualMatrix().cast<double>();
    // b_ineq.segment<16>(16) is already zero
    // w_l_lc.force().z() >= MIN_DSP_FZ
    A_ineq.block<1, 6>(32, 0) = -X_lc_0.dualMatrix().cast<double>().bottomRows<1>();
    b_ineq(32) = -MIN_DSP_FZ;
    // w_r_rc.force().z() >= MIN_DSP_FZ
    A_ineq.block<1, 6>(33, 6) = -X_rc_0.dualMatrix().cast<double>().bottomRows<1>();
    b_ineq(33) = -MIN_DSP_FZ;

    qpSolver_.problem(NB_VAR, 0, NB_CONS);
    MatrixXd A_eq(0, 0);
    VectorXd b_eq;
    b_eq.resize(0);

    bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq);
    if (!solutionFound)
    {
        std::cout << "DS Force distribution QP: solver found no solution." << std::endl;
        log << "[INFO]: > \n";
        log << "timestamp: " << theFrameInfo->time << std::endl;
        log << "desired force:" << std::endl;
        log << desiredWrench.force().transpose() << std::endl;
        log << "desired couple:" << std::endl;
        log << desiredWrench.couple().transpose() << std::endl;
        log << "A: \n";
        log << A << std::endl;
        log << "b: \n";
        log << b << std::endl;
        log << "----\n\n";
        return;
    }

    VectorXd x = qpSolver_.result();

    sva::ForceVec w_0_l(x.cast<float>().segment<3>(0), x.cast<float>().segment<3>(3));
    sva::ForceVec w_0_r(x.cast<float>().segment<3>(6), x.cast<float>().segment<3>(9));
    distribWrench_ = w_0_l + w_0_r;

    sva::ForceVec w_lc_l = X_lc_0.dualMul(w_0_l);
    sva::ForceVec w_rc_r = X_rc_0.dualMul(w_0_r);
    Vector3f e_z = {0.f, 0.f, 1.f};
    Vector2f leftCoP = (e_z.cross(w_lc_l.couple()) / w_lc_l.force()(2)).head<2>();
    Vector2f rightCoP = (e_z.cross(w_rc_r.couple()) / w_rc_r.force()(2)).head<2>();

    leftCoP.x() *= 1000.f;
    leftCoP.y() *= 1000.f;
    rightCoP.x() *= 1000.f;
    rightCoP.y() *= 1000.f;

    theLeftFootTask->targetCoP = leftCoP;
    theRightFootTask->targetCoP = rightCoP;
    theLeftFootTask->targetForce = w_lc_l.force();
    theRightFootTask->targetForce = w_rc_r.force();
}

void Stabilizer::saturateWrench(const sva::ForceVec &desiredWrench, FootTask &footTask)
{
    constexpr unsigned NB_CONS = 16;
    constexpr unsigned NB_VAR = 6;
    const sva::PTransform &X_0_c = (contactState_ == Contact::ContactState::LeftSupport) ? leftFootContact.poseW().inv() : rightFootContact.poseW().inv();
    Matrix6d Q = Matrix6d::Identity();
    Vector6d c = -desiredWrench.vector().cast<double>();

    MatrixXd A_ineq = wrenchFaceMatrix_.cast<double>() * X_0_c.dualMatrix().cast<double>();
    VectorXd b_ineq;
    b_ineq.setZero(NB_CONS);

    qpSolver_.problem(NB_VAR, 0, NB_CONS);
    MatrixXd A_eq(0, 0);
    VectorXd b_eq;
    b_eq.resize(0);

    bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq);
    if (!solutionFound)
    {
        std::cout << "SS force distribution QP: solver found no solution." << std::endl;
        return;
    }

    VectorXd x = qpSolver_.result();
    sva::ForceVec w_0(x.head<3>().cast<float>(), x.tail<3>().cast<float>());
    sva::ForceVec w_c = X_0_c.dualMul(w_0);
    Vector3f e_z = {0.f, 0.f, 1.f};
    Vector2f cop = (e_z.cross(w_c.couple()) / w_c.force()(2)).head<2>();

    footTask.targetCoP = cop;
    footTask.targetForce = w_c.force();
    distribWrench_ = w_0;
}

void Stabilizer::updateCoMTaskZMPCC()
{
    if (zmpccOnlyDS_ && contactState_ != Contact::ContactState::DoubleSupport)
    {
        zmpccCoMAccel_.setZero();
        zmpccCoMVel_.setZero();
        zmpccIntegrator_.add(Vector3f::Zero(), dt_);
    }
    else
    {
        auto distribZMP = computeZMP(distribWrench_);
        zmpccError_ = distribZMP - measuredZMP_;
        const Matrix3f &R_0_c = zmpFrame_.inv().rotation();
        Matrix3f R_c_0 = R_0_c.transpose();
        Vector3f comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.f};
        Vector3f newVel = R_c_0 * (comAdmittance.cwiseProduct(R_0_c * zmpccError_));
        Vector3f newAccel = (newVel - zmpccCoMVel_) / dt_;
        zmpccIntegrator_.add(newVel, dt_);
        zmpccCoMVel_ = newVel;
        zmpccCoMAccel_ = newAccel;
    }
    zmpccCoMOffset_ = zmpccIntegrator_.eval();
    // comTask->com(pendulum_.com() + zmpccCoMOffset_);
    // comTask->refVel(pendulum_.comd() + zmpccCoMVel_);
    // comTask->refAccel(pendulum_.comdd() + zmpccCoMAccel_);
}

void Stabilizer::updateFootForceDifferenceControl()
{
    if (contactState_ != Contact::ContactState::DoubleSupport)
    {
        dfzForceError_ = 0.f;
        dfzHeightError_ = 0.f;
        vdcHeightError_ = 0.f;
        // leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
        // rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
        theLeftFootTask->refVelB = {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
        theRightFootTask->refVelB = {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
        return;
    }
    float LFz_d = theLeftFootTask->targetWrench.force().z();
    float RFz_d = theRightFootTask->targetWrench.force().z();
    float LFz = theLeftFootTask->measuredWrench.force().z();
    float RFz = theRightFootTask->measuredWrench.force().z();
    dfzForceError_ = (LFz_d - RFz_d) - (LFz - RFz);

    float dz_ctrl = dfzAdmittance_ * dfzForceError_;
    sva::MotionVec velF = {{0.f, 0.f, 0.f}, {0.f, 0.f, dz_ctrl}};
    theLeftFootTask->refVelB = -0.5f * velF;
    theRightFootTask->refVelB = 0.5f * velF;
}

void Stabilizer::updateState(const Vector3f &com, const Vector3f &comd, const sva::ForceVec &wrench, float leftFootRatio)
{
    leftFootRatio_ = leftFootRatio;
    measuredCoM_ = com;
    measuredCoMd_ = comd;
    measuredWrench_ = wrench;
}

void Stabilizer::resetPendulum()
{
    if (pendulum_.needReset())
    {
        //! reset com height, must be done before reset pendulum.
        // const Vector3f &OPcom = theRobotModel->centerOfMass;
        // float comHeight = OPcom.z() + MotionConfig::hipHeight;
        // pendulum_.comHeight() = comHeight;

        //! reset pendulum
        Vector3f com = theFloatingBaseEstimation->WTB.translation();
        pendulum_.reset(com, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
        //! run once
        pendulum_.needReset() = false;
    }
}

void Stabilizer::configureContact()
{
    static bool once = true;
    if (once)
    {
        const float halfLength = theRobotDimensions->halfSoleLength;
        const float halfWidth = theRobotDimensions->halfSoleWidth;
        const sva::PTransform WTO = theFloatingBaseEstimation->WTO;
        leftFootContact.calcPose(*theRobotModel, halfLength, halfWidth, Contact::SurfaceType::LeftFootContact, WTO);
        rightFootContact.calcPose(*theRobotModel, halfLength, halfWidth, Contact::SurfaceType::RightFootContact, WTO);
        once = false;
    }
}
