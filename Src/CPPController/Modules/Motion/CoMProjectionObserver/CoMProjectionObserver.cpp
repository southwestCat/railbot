#include "CoMProjectionObserver.h"
#include "SpaceVecAlg/PTransform.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Module/ModuleManager.h"

void CoMProjectionObserver::update()
{
    UPDATE_REPRESENTATION(RobotModel);
}

void CoMProjectionObserver::update(CoMProjectionEstimation &o)
{
    update();

    const Pose3f soleLeft = theRobotModel->soleLeft;
    const Pose3f soleRight = theRobotModel->soleRight;
    //! left contact foot surface.
    const Pose3f soleLeftSurface = soleLeft + Vector3f(30.f, 5.62f, 0.f);
    //! right contact foot surface.
    const Pose3f soleRightSurface = soleRight + Vector3f(30.f, -5.62f, 0.f);

    sva::PTransform X_o_ls_mm = sva::PTransform(soleLeftSurface.rotation, soleLeftSurface.translation);
    sva::PTransform X_o_ls = X_o_ls_mm.toMeter();
    sva::PTransform X_o_rs_mm = sva::PTransform(soleRightSurface.rotation, soleRightSurface.translation);
    sva::PTransform X_o_rs = X_o_rs_mm.toMeter();
    //! surface frame in BH-Robot frame.
    sva::PTransform X_o_surface = sva::interpolate(X_o_ls, X_o_rs, 0.5);

    //! com in BH-Robot frame.
    Vector3f com = theRobotModel->centerOfMass;
    sva::PTransform X_o_com_mm = sva::PTransform(Matrix3f::Identity(), com);
    sva::PTransform X_o_com = X_o_com_mm.toMeter();

    //! com in surface frame.
    sva::PTransform X_surface_com = X_o_surface.inv() * X_o_com;
    Vector3f comEstimated = X_surface_com.translation();

    //! estimated com projection
    o.estimatedCoP = {comEstimated.x() * 1000.f, comEstimated.y() * 1000.f}; //< convert to mm

    //! BH-Robot frame in world frame.
    sva::PTransform WTO_mm = theFloatingBaseEstimation->WTO;
    sva::PTransform WTO = WTO_mm.toMeter();
    //! world frame in surface frame.
    sva::PTransform STW = X_o_surface.inv() * WTO.inv();
    //! measured com projection --> center of pressure.
    Vector3f cop = theNetWrenchEstimation->netZMP;
    //! measured cop in world frame.
    sva::PTransform X_W_cop = {Matrix3f::Identity(), cop};

    Vector3f copMeasured = (STW * X_W_cop).translation();

    o.measuredCoP = {copMeasured.x() * 1000.f, copMeasured.y() * 1000.f};
}
