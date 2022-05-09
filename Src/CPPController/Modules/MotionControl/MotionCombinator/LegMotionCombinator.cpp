#include "LegMotionCombinator.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Motion/MotionUtilities.h"

void LegMotionCombinator::update()
{
    UPDATE_REPRESENTATION(JointAngles);
    UPDATE_REPRESENTATION(LegMotionSelection);
    UPDATE_REPRESENTATION(StandEngineOuptut);
    UPDATE_REPRESENTATION(SpecialActionEngineOutput);
}

void LegMotionCombinator::update(LegJointRequest &legJointRequest)
{
    update();

    const JointRequest *jointRequest[MotionRequest::numOfMotions];
    jointRequest[MotionRequest::walk] = theWalkingEngineOutput;
    jointRequest[MotionRequest::stand] = theStandEngineOuptut;
    jointRequest[MotionRequest::specialAction] = theSpecialActionEngineOutput;

    MotionUtilities::copy(*jointRequest[theLegMotionSelection->targetMotion], legJointRequest, *theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);

    if (theLegMotionSelection->ratios[theLegMotionSelection->targetMotion] == 1.f)
        lastJointAngles = *theJointAngles;
    else
    {
        const bool interpolateStiffness = theLegMotionSelection->targetMotion == MotionRequest::specialAction ? false : true;
        for (int i = 0; i < MotionRequest::numOfMotions; i++)
        {
            if (i != theLegMotionSelection->targetMotion && theLegMotionSelection->ratios[i] > 0.f)
            {
                MotionUtilities::interpolate(*jointRequest[i], *jointRequest[theLegMotionSelection->targetMotion], theLegMotionSelection->ratios[i],
                legJointRequest, interpolateStiffness, *theStiffnessSettings, lastJointAngles, Joints::firstLegJoint, Joints::rAnkleRoll);
            }
        }
    }
}
