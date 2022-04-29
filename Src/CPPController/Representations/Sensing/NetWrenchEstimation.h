#pragma once

#include "SpaceVecAlg/ForceVec.h"

class NetWrenchEstimation
{
public:
    sva::ForceVec netWrench;
    Vector3f netZMP;

    sva::ForceVec wrenchLeft;
    sva::ForceVec wrenchRight;
};