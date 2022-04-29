#pragma once

#include "Tools/Math/Pose3f.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Limbs.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"

namespace ForwardKinematic
{
    void calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, std::array<Pose3f, Limbs::numOfLimbs>& limbs);
    void calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions, std::array<Pose3f, Limbs::numOfLimbs>& limbs);
    void calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, std::array<Pose3f, Limbs::numOfLimbs>& limbs);
}