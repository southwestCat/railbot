#include "ComplianceController.h"
#include "Tools/Module/ModuleManager.h"

void ComplianceController::update()
{
    UPDATE_REPRESENTATION(CoMProjectionEstimation);
}

void ComplianceController::update(ComplianceJointRequest &o)
{
    update();

    printf(">\n");

    float zmpx = theNetWrenchEstimation->netZMP.x() * 1000.f;
    float zmpy = theNetWrenchEstimation->netZMP.y() * 1000.f;
    printf("%3.3f %3.3f\n", zmpx, zmpy);

    // float ecopx = theCoMProjectionEstimation->estimatedCoP.x();
    // float ecopy = theCoMProjectionEstimation->estimatedCoP.y();
    // float mcopx = theCoMProjectionEstimation->measuredCoP.x();
    // float mcopy = theCoMProjectionEstimation->measuredCoP.y();
    // printf("estimate: %3.3f %3.3f\n", ecopx, ecopy);
    // printf("measured: %3.3f %3.3f\n", mcopx, mcopy);
    printf("----\n\n");
}
