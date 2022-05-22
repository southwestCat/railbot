#include "FsrFilteredDataProvider.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Math/Constants.h"

void FsrFilteredDataProvider::update()
{
    UPDATE_REPRESENTATION(FsrSensorData);
}

void FsrFilteredDataProvider::update(FsrFilteredData &fsr)
{
    update();

    float lsum = theFsrSensorData->totals[Legs::left];
    float rsum = theFsrSensorData->totals[Legs::right];
    float sum = lsum + rsum;

    if (sum > 2.f)
    {
        float rate = theMassCalibration->totalMass / 1000.f * Constants::g_1000 / sum;
        for (int leg = 0; leg < Legs::numOfLegs; leg++)
        {
            fsr.totals[leg] = 0.f;
            for (int sensor = 0; sensor < FsrSensors::numOfFsrSensors; sensor++)
            {
                fsr.pressures[leg][sensor] = theFsrSensorData->pressures[leg][sensor] * rate;
                fsr.totals[leg] += fsr.pressures[leg][sensor];
            }
        }
    }
    else
    {
        for (int leg = 0; leg < Legs::numOfLegs; leg++)
        {
            for (int sensor = 0; sensor < FsrSensors::numOfFsrSensors; sensor++)
            {
                fsr.pressures[leg][sensor] = theFsrSensorData->pressures[leg][sensor] * 23.3f;
            }
        }
    }

    // float lfl = fsr.pressures[Legs::left][FsrSensors::fl];
    // float lfr = fsr.pressures[Legs::left][FsrSensors::fr];
    // float lbl = fsr.pressures[Legs::left][FsrSensors::bl];
    // float lbr = fsr.pressures[Legs::left][FsrSensors::br];

    // float rfl = fsr.pressures[Legs::right][FsrSensors::fl];
    // float rfr = fsr.pressures[Legs::right][FsrSensors::fr];
    // float rbl = fsr.pressures[Legs::right][FsrSensors::bl];
    // float rbr = fsr.pressures[Legs::right][FsrSensors::br];

    // float l_sum = lfl + lfr + lbl + lbr;
    // float r_sum = rfl + rfr + rbl + rbr;

    // printf("----------foot sensors----------\n");
    // printf("   left       right\n");
    // printf("+--------+ +--------+\n");
    // printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  front\n", lfl, lfr, rfl, rfr);
    // printf("|        | |        |\n");
    // printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  back\n", lbl, lbr, rbl, rbr);
    // printf("left: %3.3f, right: %3.3f\n", l_sum, r_sum);
    // printf("total: %3.3f \n", l_sum + r_sum);
    // printf("+--------+ +--------+\n");
}
