#pragma once

#include "SpaceVecAlg/PTransform.h"

class FootTask
{
public:
    sva::ForceVec targetWrench;
    sva::ForceVec measuredWrench;
    sva::PTransform targetPose;
    sva::PTransform surfacePose; //< not used
    sva::MotionVec refVelB;

    Vector2f targetCoP;
    Vector3f targetForce;
};