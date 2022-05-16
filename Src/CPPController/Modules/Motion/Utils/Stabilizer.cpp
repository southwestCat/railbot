#include "Stabilizer.h"
#include "Modules/Motion/Utils/clamp.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/ModuleManager.h"

Stabilizer::Stabilizer()
    : dcmIntegrator_(5.f), dcmDerivator_(1.f) {}

void Stabilizer::update()
{
    UPDATE_REPRESENTATION(RobotModel);
    UPDATE_REPRESENTATION(FsrSensorData);

    leftFootContact.calcPose(*theRobotModel, theRobotDimensions->halfSoleLength, theRobotDimensions->halfSoleWidth, Contact::SurfaceType::LeftFootContact, theFloatingBaseEstimation->WTO);
    rightFootContact.calcPose(*theRobotModel, theRobotDimensions->halfSoleLength, theRobotDimensions->halfSoleWidth, Contact::SurfaceType::RightFootContact, theFloatingBaseEstimation->WTO);
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
}

Vector3f Stabilizer::computeZMP() const
{
    const Pose3f &soleL = theRobotModel->soleLeft;
    const Pose3f &soleR = theRobotModel->soleRight;
    const FsrSensorData &fsr = *theFsrSensorData;

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
    printf("In Stabilizer. \n");
}
