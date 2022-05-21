#pragma once

#include "Pendulum.h"
#include "Contact.h"
#include "Modules/Motion/Utils/ExponentialMovingAverage.h"
#include "Modules/Motion/Utils/StationaryOffsetFilter.h"
#include "Modules/Motion/Utils/LeakyIntegrator.h"
#include "SpaceVecAlg/PTransform.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/MotionControl/FootTask.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"

#include <vector>
#include <eigen-quadprog/QuadProg.h>

class StabilizerBase
{
public:
    REQUIRES_REPRESENTATION(RobotModel);
    // REQUIRES_REPRESENTATION(FsrSensorData);
    REQUIRES_REPRESENTATION(FsrFilteredData);
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(RobotDimensions);
    USES_REPRESENTATION(MassCalibration);
    USES_REPRESENTATION(FloatingBaseEstimation);
    USES_REPRESENTATION(NetWrenchEstimation);

    MODIFIES_REPRESENTATION(LeftFootTask);
    MODIFIES_REPRESENTATION(RightFootTask);
};

class Stabilizer : public StabilizerBase
{
public:
    /* Maximum angular velocities for foot damping control. */
    static constexpr double MAX_FDC_RX_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RY_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RZ_VEL = 0.2; // [rad] / [s]

    /* Maximum gains for HRP4LIRMM in standing static equilibrium. */
    static constexpr double MAX_COM_XY_ADMITTANCE = 20.;
    static constexpr double MAX_COM_Z_ADMITTANCE = 0.01;
    static constexpr double MAX_COP_ADMITTANCE = 0.1;
    static constexpr double MAX_DCM_I_GAIN = 100.;
    static constexpr double MAX_DCM_P_GAIN = 20.;
    static constexpr double MIN_DCM_P_GAIN = 1.;
    static constexpr double MAX_DCM_D_GAIN = 2.;
    static constexpr double MAX_DFZ_ADMITTANCE = 5e-4;
    static constexpr double MAX_ZMP_GAIN = 20.;

    /* Avoid low-pressure targets too close to contact switches */
    static constexpr double MIN_DS_PRESSURE = 15.; // [N]
    static constexpr double MIN_DSP_FZ = 15.;

    /* Saturate integrator in case of windup */
    static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; // [m]
    static constexpr double MAX_ALTCC_COM_OFFSET = 0.05;  // [m]
    static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05;  // [m]

    enum class TemplateModel
    {
        LinearInvertedPendulum,
        VariableHeightInvertedPendulum
    };

    Stabilizer();

    void disable();

    Vector3f computeZMP() const;
    Vector3f computeZMP(const sva::ForceVec &wrench) const;

    void configure();

    bool detectTouchDown() { return true; }

    void reconfigure();

    void removeTasks() {}

    void reset();

    void run();

    void seekTouchDown() {}

    void setContact() {}

    void setSwingFoot() {}

    Contact::ContactState contactState() const { return contactState_; }

    void contactState(Contact::ContactState c) { contactState_ = c; }

    void updateMass(float m) { mass_ = m; }

    void updateState(const Vector3f &com, const Vector3f &comd, const sva::ForceVec &wrench, float leftFootRatio);

    void wrenchFaceMatrix(float halfLength, float halfWidth, float friction)
    {
        float X = halfLength;
        float Y = halfWidth;
        float mu = friction;
        wrenchFaceMatrix_ << 0, 0, 0, -1, 0, -mu,
            0, 0, 0, +1, 0, -mu,
            0, 0, 0, 0, -1, -mu,
            0, 0, 0, 0, +1, -mu,
            -1, 0, 0, 0, 0, -Y,
            +1, 0, 0, 0, 0, -Y,
            0, -1, 0, 0, 0, -X,
            0, +1, 0, 0, 0, -X,
            +mu, +mu, -1, -Y, -X, -(X + Y) * mu,
            +mu, -mu, -1, -Y, +X, -(X + Y) * mu,
            -mu, +mu, -1, +Y, -X, -(X + Y) * mu,
            -mu, -mu, -1, +Y, +X, -(X + Y) * mu,
            +mu, +mu, +1, +Y, +X, -(X + Y) * mu,
            +mu, -mu, +1, +Y, -X, -(X + Y) * mu,
            -mu, +mu, +1, -Y, +X, -(X + Y) * mu,
            -mu, -mu, +1, -Y, -X, -(X + Y) * mu;
    }

    Vector3f zmp() const { return computeZMP(distribWrench_); }

    const sva::PTransform &zmpFrame() const { return zmpFrame_; }

    const std::vector<Vector3f> &zmpPolygon() { return zmpPolygon_; }

private:
    class FDQPWeights
    {
    public:
        float ankleTorqueSqrt;
        float netWrenchSqrt;
        float forceRatioSqrt;
    };

private:
    void update();
    void checkGains();
    void checkInTheAir();
    sva::ForceVec computeDesiredWrench();
    sva::ForceVec computeLIPDesiredWrench();
    sva::ForceVec computeVHIPDesiredWrench();
    void distributeWrench(const sva::ForceVec &desiredWrench);
    void distributeWrenchDS(const sva::ForceVec &desiredWrench);
    void distributeWrenchSS(const sva::ForceVec &desiredWrench);
    void saturateWrench(const sva::ForceVec &desiredWrench, FootTask &footTask);
    //! Update CoM task with ZMP Compensation Control.
    void updateCoMTaskZMPCC();
    void updateSupportFootGains();
    void updateCoMAdmittanceControl();
    void updateCoMZMPCC();
    void updateCoMAltitude();
    void updateFootForceDifferenceControl();
    void updateZMPFrame();
    sva::ForceVec contactAdmittance()
    {
        return {{copAdmittance_.y(), copAdmittance_.x(), 0.f}, {0.f, 0.f, 0.f}};
    }
    void resetPendulum();
    void configureContact();

public:
    Contact leftFootContact;
    Contact rightFootContact;

private:
    Contact::ContactState contactState_ = Contact::ContactState::DoubleSupport;
    HrepXf zmpArea_;
    Eigen::QuadProgDense leastSquares_;
    Eigen::QuadProgDense qpSolver_;
    Matrix16x6f wrenchFaceMatrix_;
    Vector2f copAdmittance_ = Vector2f::Zero();
    Vector3f altccCoMAccel_ = Vector3f::Zero();
    Vector3f altccCoMOffset_ = Vector3f::Zero();
    Vector3f altccCoMVel_ = Vector3f::Zero();
    Vector3f altccError_ = Vector3f::Zero();
    Vector3f comAdmittance_ = Vector3f::Zero();
    Vector3f comOffset_ = Vector3f::Zero();
    Vector3f comStiffness_ = {1000.f, 1000.f, 100.f}; /**< Stiffness of CoM IK task */
    Vector3f dcmAverageError_ = Vector3f::Zero();
    Vector3f dcmError_ = Vector3f::Zero();
    Vector3f dcmVelError_ = Vector3f::Zero();
    Vector3f measuredCoM_ = Vector3f::Zero();
    Vector3f measuredCoMd_ = Vector3f::Zero();
    Vector3f measuredZMP_ = Vector3f::Zero();
    Vector3f zmpError_ = Vector3f::Zero();
    Vector3f vhipDCM_ = Vector3f::Zero();
    Vector3f vhipZMP_ = Vector3f::Zero();
    Vector3f zmpccCoMAccel_ = Vector3f::Zero();
    Vector3f zmpccCoMOffset_ = Vector3f::Zero();
    Vector3f zmpccCoMVel_ = Vector3f::Zero();
    Vector3f zmpccError_ = Vector3f::Zero();
    Vector4f polePlacement_ = {-10.f, -5.f, -1.f, 10.f};
    ExponentialMovingAverage dcmIntegrator_;
    FDQPWeights fdqpWeights_;
    LeakyIntegrator<Vector3f> altccIntegrator_;
    LeakyIntegrator<Vector3f> zmpccIntegrator_;
    StationaryOffsetFilter dcmDerivator_;
    TemplateModel model_ = TemplateModel::LinearInvertedPendulum;
    bool inTheAir_ = false;
    bool zmpccOnlyDS_ = true;
    Pendulum pendulum_;
    // const mc_rbdyn::Robot & controlRobot_;
    float comWeight_ = 1000.f;
    float contactWeight_ = 100000.f;
    float dcmGain_ = 1.25f;
    float dcmDerivGain_ = 0.f;
    float dcmIntegralGain_ = 2.5f;
    float dcmPropGain_ = 0.f;
    float dfzAdmittance_ = 1e-4;
    float dfzDamping_ = 0.f;
    float dfzForceError_ = 0.f;
    float dfzHeightError_ = 0.f;
    float distribLambda_ = 0.f;
    const float &dt_ = Blackboard::getInstance().dt();
    float fdqpRunTime_ = 0.f;
    float lambdaMax_ = 30.f;
    float lambdaMin_ = 0.f;
    float leftFootRatio_ = 0.5f;
    float logMeasuredDFz_ = 0.f;
    float logMeasuredSTz_ = 0.f;
    float logTargetDFz_ = 0.f;
    float logTargetSTz_ = 0.f;
    float mass_;
    float measuredLambda_ = 0.f;
    float runTime_ = 0.f;
    float swingFootStiffness_ = 2000.f;
    float swingFootWeight_ = 500.f;
    float vdcDamping_ = 0.f;
    float vdcFrequency_ = 1.f;
    float vdcHeightError_ = 0.f;
    float vdcStiffness_ = 1000.f;
    float vdcZPos_ = 0.f;
    float vfcZCtrl_ = 0.f;
    float vhipLambda_ = 0.f;
    float vhipOmega_ = 0.f;
    float vhipRunTime_ = 0.f;
    // mc_rtc::Configuration config_;
    std::vector<Vector3f> zmpPolygon_;
    std::vector<std::string> comActiveJoints_;
    sva::ForceVec distribWrench_ = sva::ForceVec::Zero();
    sva::ForceVec measuredWrench_;
    sva::MotionVec contactDamping_;
    sva::MotionVec contactStiffness_;
    sva::PTransform zmpFrame_;
};