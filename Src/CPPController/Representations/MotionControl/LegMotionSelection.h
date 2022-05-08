#pragma once

#include "MotionRequest.h"
#include <array>

class LegMotionSelection
{
public:
    MotionRequest::Motion targetMotion = MotionRequest::specialAction;
    std::array<float, MotionRequest::numOfMotions> ratios;
};