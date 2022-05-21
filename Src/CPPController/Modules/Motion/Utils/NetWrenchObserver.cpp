#include "NetWrenchObserver.h"
#include "Tools/Module/ModuleManager.h"
#include "Modules/Motion/MotionConfigure.h"

void NetWrenchObserver::update()
{
    UPDATE_REPRESENTATION(FsrFilteredData);
    // UPDATE_REPRESENTATION(FsrSensorData);
    UPDATE_REPRESENTATION(RobotModel);
}

void NetWrenchObserver::update(const Contact &contact)
{
    update();
    updateNetWrench();
    updateZMP(contact);
}

void NetWrenchObserver::updateNetWrench()
{
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

    const float &Fltotal = fsr.totals[Legs::left];
    const float &Frtotal = fsr.totals[Legs::right];

    //! Position
    const Vector2f &Plfl = theRobotDimensions->leftFsrPositions[FsrSensors::fl];
    const Vector2f &Plfr = theRobotDimensions->leftFsrPositions[FsrSensors::fr];
    const Vector2f &Plbl = theRobotDimensions->leftFsrPositions[FsrSensors::bl];
    const Vector2f &Plbr = theRobotDimensions->leftFsrPositions[FsrSensors::br];

    const Vector2f &Prfl = theRobotDimensions->rightFsrPositions[FsrSensors::fl];
    const Vector2f &Prfr = theRobotDimensions->rightFsrPositions[FsrSensors::fr];
    const Vector2f &Prbl = theRobotDimensions->rightFsrPositions[FsrSensors::bl];
    const Vector2f &Prbr = theRobotDimensions->rightFsrPositions[FsrSensors::br];

    const sva::PTransform &WTO = theFloatingBaseEstimation->WTO;

    //! Sole left in BH robot frame.
    const Pose3f &soleLeft = theRobotModel->soleLeft;
    sva::PTransform OTSL = {soleLeft.rotation, soleLeft.translation};
    //! Sole right in BH robot frame.
    const Pose3f &soleRight = theRobotModel->soleRight;
    sva::PTransform OTSR = {soleRight.rotation, soleRight.translation};

    //! Wrench in left sole frame.
    sva::ForceVec w_sole_lfl = calcWrench(Flfl, Plfl);
    sva::ForceVec w_sole_lfr = calcWrench(Flfr, Plfr);
    sva::ForceVec w_sole_lbl = calcWrench(Flbl, Plbl);
    sva::ForceVec w_sole_lbr = calcWrench(Flbr, Plbr);
    sva::ForceVec w_sole_lsum = w_sole_lfl + w_sole_lfr + w_sole_lbl + w_sole_lbr;
    // Vector3f force_left = w_sole_lsum.force();
    
    //! Wrench in right sole frame
    sva::ForceVec w_sole_rfl = calcWrench(Frfl, Prfl);
    sva::ForceVec w_sole_rfr = calcWrench(Frfr, Prfr);
    sva::ForceVec w_sole_rbl = calcWrench(Frbl, Prbl);
    sva::ForceVec w_sole_rbr = calcWrench(Frbr, Prbr);
    sva::ForceVec w_sole_rsum = w_sole_rfl + w_sole_rfr + w_sole_rbl + w_sole_rbr;
    // Vector3f force_right = w_sole_rsum.force();

    //! lsum wrench in BH robot frame.
    sva::ForceVec w_O_lsum = OTSL.dualMul(w_sole_lsum);
    //! lsum wrench in world frame.
    sva::ForceVec w_W_lsum = WTO.dualMul(w_O_lsum);

    //! rsum wrench in BH robot frame.
    sva::ForceVec w_O_rsum = OTSR.dualMul(w_sole_rsum);
    //! rsum wrench in world frame.
    sva::ForceVec w_W_rsum = WTO.dualMul(w_O_rsum);
    
    //! wrench in world frame.
    netWrench_ = w_W_lsum + w_W_rsum;

    theNetWrenchEstimation->netWrench = netWrench_;
    theNetWrenchEstimation->wrenchLeft = w_sole_lsum;
    theNetWrenchEstimation->wrenchRight = w_sole_rsum;
}

void NetWrenchObserver::updateZMP(const Contact &contact)
{
    const Vector3f &force = netWrench_.force();
    const Vector3f &moment_0 = netWrench_.couple();
    Vector3f moment_p = moment_0 - contact.p().cross(force);
    if (force.dot(force) > MotionConfig::minMeasuredPressure)
    {
        netZMP_ = contact.p() + contact.normal().cross(moment_p) / contact.normal().dot(force);
    }
    else
    {
        netZMP_ = {-1.f, -1.f, -1.f};
    }

    theNetWrenchEstimation->netZMP = netZMP_;
}

sva::ForceVec NetWrenchObserver::calcWrench(float f, Vector2f p)
{
    Vector3f force = {0.f, 0.f, f};
    Vector3f position = {p.x() / 1000.f, p.y() / 1000.f, 0.f};
    return {position.cross(force), force};
}
