#pragma once

#include "Arms.h"
#include "Legs.h"
#include <vector>

namespace Joints
{
    enum Joint
    {
        headYaw,
        headPitch,

        firstArmJoint,
        firstLeftArmJoint = firstArmJoint,

        lShoulderPitch = firstLeftArmJoint,
        lShoulderRoll,
        lElbowYaw,
        lElbowRoll,
        lWristYaw,
        lHand, //< not an Angle, instead %

        firstRightArmJoint,

        rShoulderPitch = firstRightArmJoint,
        rShoulderRoll,
        rElbowYaw,
        rElbowRoll,
        rWristYaw,
        rHand, //< not an Angle, instead %

        firstLegJoint,
        firstLeftLegJoint = firstLegJoint,

        lHipYawPitch = firstLeftLegJoint,
        lHipRoll,
        lHipPitch,
        lKneePitch,
        lAnklePitch,
        lAnkleRoll,

        firstRightLegJoint,

        rHipYawPitch = firstRightLegJoint, //< not a joint in the real nao
        rHipRoll,
        rHipPitch,
        rKneePitch,
        rAnklePitch,
        rAnkleRoll,
        numOfJoints
    };

    using stdVectorJoint = std::vector<Joint>;

    enum JointArmVarieties
    {
        shoulderPitch,
        shoulderRoll,
        elbowYaw,
        elbowRoll,
        wristYaw,
        hand,
        numOfJointArmVarietiess
    };

    enum JointLegVarieties
    {
        hipYawPitch, //< not a joint in the real nao
        hipRoll,
        hipPitch,
        kneePitch,
        anklePitch,
        ankleRoll,
        numOfJointLegVarietiess
    };

    inline Joint combine(const Arms::Arm arm, const JointArmVarieties jointV)
    {
        static const unsigned offset[2] = {0u, firstRightArmJoint - firstLeftArmJoint};
        return Joint(firstArmJoint + jointV + offset[arm]);
    }

    inline Joint combine(const Legs::Leg leg, const JointLegVarieties jointV)
    {
        static const unsigned offset[2] = {0u, firstRightLegJoint - firstLeftLegJoint};
        return Joint(firstLeftLegJoint + jointV + offset[leg]);
    }
}