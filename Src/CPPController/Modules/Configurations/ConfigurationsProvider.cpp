#include <fstream>

#include "ConfigurationsProvider.h"
#include "json.hpp"

using nlohmann::json;
using std::ifstream;

void ConfigurationsProvider::update(IMUCalibration &imu)
{
    ifstream f("Config/IMUCalibration.json");
    if (!f)
    {
        printf("[ERROR] Config/IMUCalibration.json, file not exists !\n");
        f.close();
        return;
    }
    json j;
    f >> j;
    imu.fromJson(j);
    f.close();
}