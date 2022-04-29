#pragma once

#include "SpaceVecAlg/PTransform.h"

struct FloatingBaseEstimation
{
    sva::PTransform WTB; //< com coordinates in world frame.
    sva::PTransform WTO; //< robot frame in world frame.
    sva::PTransform OTA; //< anchor frame in BH robot frame.
};