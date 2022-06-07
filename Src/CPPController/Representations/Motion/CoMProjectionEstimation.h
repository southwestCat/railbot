#pragma once

#include "Tools/Math/Eigen.h"

class CoMProjectionEstimation
{
public:
    Vector2f estimatedCoP;
    Vector2f measuredCoP;

    Vector2f estimatedCoPNormalized;
    Vector2f measuredCoPNormalized;
};