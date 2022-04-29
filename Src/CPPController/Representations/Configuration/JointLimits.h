#pragma once

#include "Tools/RobotParts/Joints.h"
#include "Tools/Range.h"

class JointLimits
{
public:
    JointLimits()
    {
        limits.fill(150_deg);
        limits[Joints::headYaw] = Rangea(-119.5_deg, 119.5_deg);
        limits[Joints::headPitch] = Rangea(-38.5_deg, 29.5_deg);
        limits[Joints::lShoulderPitch] = Rangea(-119.5_deg, 119.5_deg);
        limits[Joints::lShoulderRoll] = Rangea(-18_deg, 76_deg);
        limits[Joints::lElbowYaw] = Rangea(-119.5_deg, 119.5_deg);
        limits[Joints::lElbowRoll] = Rangea(-88.5_deg, -2_deg);
        limits[Joints::lWristYaw] = Rangea(-104.5_deg, 104.5_deg);
        limits[Joints::lHand] = Rangea(0_deg, 57.2958_deg);
        limits[Joints::rShoulderPitch] = Rangea(-119.5_deg, 119.5_deg);
        limits[Joints::rShoulderRoll] = Rangea(-76_deg, 18_deg);
        limits[Joints::rElbowYaw] = Rangea(-119.5_deg, 119.5_deg);
        limits[Joints::rElbowRoll] = Rangea(2_deg, 88.5_deg);
        limits[Joints::rWristYaw] = Rangea(-104.5_deg, 104.5_deg);
        limits[Joints::rHand] = Rangea(0_deg, 57.2958_deg);
        limits[Joints::lHipYawPitch] = Rangea(-65.62_deg, 42.44_deg);
        limits[Joints::lHipRoll] = Rangea(-21.74_deg, 45.29_deg);
        limits[Joints::lHipPitch] = Rangea(-88_deg, 27.73_deg);
        limits[Joints::lKneePitch] = Rangea(-5.29_deg, 121.04_deg);
        limits[Joints::lAnklePitch] = Rangea(-68.15_deg, 52.86_deg);
        limits[Joints::lAnkleRoll] = Rangea(-22.79_deg, 44.06_deg);
        limits[Joints::rHipYawPitch] = Rangea(-65.62_deg, 42.44_deg);
        limits[Joints::rHipRoll] = Rangea(-45.29_deg, 21.74_deg);
        limits[Joints::rHipPitch] = Rangea(-88_deg, 27.73_deg);
        limits[Joints::rKneePitch] = Rangea(-5.9_deg, 121.47_deg);
        limits[Joints::rAnklePitch] = Rangea(-67.97_deg, 53.4_deg);
        limits[Joints::rAnkleRoll] = Rangea(-44.06_deg, 22.8_deg);
    }

    std::array<Rangea, Joints::numOfJoints> limits;
};