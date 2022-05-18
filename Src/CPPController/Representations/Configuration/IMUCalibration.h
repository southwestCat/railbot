#pragma once

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
};