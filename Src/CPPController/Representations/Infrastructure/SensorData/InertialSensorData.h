#pragma once

#include "Tools/Math/Eigen.h"

class InertialSensorData
{
public:
    Vector3a gyro = Vector3a::Zero();
    Vector3f acc = Vector3f::Zero();
    Vector3a angle = Vector3a::Zero();
};