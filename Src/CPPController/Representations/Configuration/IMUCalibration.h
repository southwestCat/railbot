#pragma once

#include "Modules/Configurations/json.hpp"
#include "Tools/Math/Eigen.h"

class IMUCalibration
{
public:
    IMUCalibration()
    {
        rotation = AngleAxisf::Identity();
        gyroFactor << 1.f, 1.f, 1.f;

        // EllipsoldCalibrator
        accFactor << 1.f, 1.f, 1.f;
        accBias << 0.f, 0.f, 0.f;
    }

    AngleAxisf rotation;
    Vector3f gyroFactor;
    Vector3f accBias;
    Vector3f accFactor;

    void fromJson(const nlohmann::json &j)
    {
        std::array<float, 3> accB;
        std::array<float, 3> accF;
        accB = j.at("accBias").get<std::array<float, 3>>();
        accF = j.at("accFactor").get<std::array<float, 3>>();
        accBias << accB[0], accB[1], accB[2];
        accFactor << accF[0], accF[1], accF[2];
    }
};