#include "CoMProjectionObserver.h"
#include "SpaceVecAlg/PTransform.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Module/ModuleManager.h"

void CoMProjectionObserver::update()
{
    UPDATE_REPRESENTATION(FsrSensorData);
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

    sva::PTransform X_o_ls = sva::PTransform(soleLeftSurface.rotation, soleLeftSurface.translation);
    sva::PTransform X_o_rs = sva::PTransform(soleRightSurface.rotation, soleRightSurface.translation);
    //! surface frame in BH-Robot frame.
    sva::PTransform X_o_surface = sva::interpolate(X_o_ls, X_o_rs, 0.5);

    //! com in BH-Robot frame.
    Vector3f com = theRobotModel->centerOfMass;
    sva::PTransform X_o_com = sva::PTransform(Matrix3f::Identity(), com);

    //! com in surface frame.
    sva::PTransform X_surface_com = X_o_surface.inv() * X_o_com;
    Vector3f comEstimated = X_surface_com.translation();

    //! estimated com projection
    o.estimatedCoP = {comEstimated.x(), comEstimated.y()};

    //! BH-Robot frame in world frame.
    const sva::PTransform WTO = theFloatingBaseEstimation->WTO;
    //! world frame in surface frame.
    sva::PTransform STW = X_o_surface.inv() * WTO.inv();
    //! measured com projection --> center of pressure.
    Vector3f cop = theNetWrenchEstimation->netZMP;
    //! measured cop in world frame.
    sva::PTransform X_W_cop = {Matrix3f::Identity(), cop};

    Vector3f copMeasured = (STW * X_W_cop).translation();

    o.measuredCoP = {copMeasured.x(), copMeasured.y()};

    std::cout << copMeasured.transpose() << std::endl;
}
