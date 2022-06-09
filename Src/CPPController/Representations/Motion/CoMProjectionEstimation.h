#pragma once

#include "Tools/Math/Eigen.h"

class CoMProjectionEstimation
{
public:
    const float normalizedX = 80.f;
    const float normalizedY = 100.f;

    Vector2f estimatedCoP;
    Vector2f measuredCoP;

    Vector2f estimatedCoPNormalized;
    Vector2f measuredCoPNormalized;
};