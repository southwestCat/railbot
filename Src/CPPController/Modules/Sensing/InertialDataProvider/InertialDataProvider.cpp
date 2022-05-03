#include "InertialDataProvider.h"
#include "Tools/Module/ModuleManager.h"

InertialDataProvider::InertialDataProvider()
{
    beta = 0.1f;
    q0 = 1.f;
    q1 = 0.f;
    q2 = 0.f;
    q3 = 0.f;
}

void InertialDataProvider::update()
{
    UPDATE_REPRESENTATION(InertialSensorData);
}

void InertialDataProvider::update(InertialData &inertialData)
{
    update();
}
