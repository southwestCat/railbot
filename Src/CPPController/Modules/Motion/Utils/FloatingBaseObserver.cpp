#include "FloatingBaseObserver.h"
#include "Tools/Module/ModuleManager.h"

void FloatingBaseObserver::update()
{
    UPDATE_REPRESENTATION(InertialData);
    UPDATE_REPRESENTATION(RobotModel);
}

void FloatingBaseObserver::update(FloatingBaseEstimation &f)
{
    update();
    run(f);
}

void FloatingBaseObserver::reset(const sva::PTransform &X_o_fb)
{
    orientation_ = X_o_fb.rotation();
    position_ = X_o_fb.translation();
}

void FloatingBaseObserver::run(FloatingBaseEstimation &f)
{
    estimateOrientation();
    estimatePosition(f);
    updateRobot(f);
}

void FloatingBaseObserver::estimateOrientation()
{
    sva::PTransform OTA = getAnchorFrame();
    Matrix3f R_forward = OTA.inv().rotation();
    Matrix3f R_measured = theInertialData->orientation3D.matrix().eval();
    orientation_ = R_measured * R_forward.transpose();
}

void FloatingBaseObserver::estimatePosition(FloatingBaseEstimation &f)
{
    const Vector3f &com = theRobotModel->centerOfMass;
    sva::PTransform &OTA = f.OTA;
    OTA = getAnchorFrame();
    sva::PTransform OTB = sva::PTransform(Matrix3f::Identity(), com);
    sva::PTransform ATB = OTA.inv() * OTB;
    Vector3f APB = ATB.translation();
    position_ = Vector3f(0.f, 0.f, 0.f) + orientation_ * APB;
}

sva::PTransform FloatingBaseObserver::getAnchorFrame()
{
    const Pose3f &soleLeft = theRobotModel->soleLeft;
    const Pose3f &soleRight = theRobotModel->soleRight;
    sva::PTransform X_0_l = sva::PTransform(soleLeft.rotation, soleLeft.translation);
    sva::PTransform X_0_r = sva::PTransform(soleRight.rotation, soleRight.translation);
    return sva::interpolate(X_0_r, X_0_l, leftFootRatio_);
}

void FloatingBaseObserver::updateRobot(FloatingBaseEstimation &f)
{
    sva::PTransform &WTB = f.WTB;
    WTB = sva::PTransform(orientation_, position_);

    const sva::PTransform &OTA = f.OTA;
    sva::PTransform WTA = {orientation_, {0.f, 0.f, 0.f}};
    sva::PTransform &WTO = f.WTO;
    WTO = WTA * OTA.inv();
}
