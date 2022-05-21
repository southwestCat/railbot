#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Range.h"

class FsrFilteredDataProviderBase
{
public:
    REQUIRES_REPRESENTATION(FsrSensorData);

    USES_REPRESENTATION(MassCalibration);
};

class FsrFilteredDataProvider : public FsrFilteredDataProviderBase
{
public:
    void update(FsrFilteredData &fsr);

private:
    void update();
};