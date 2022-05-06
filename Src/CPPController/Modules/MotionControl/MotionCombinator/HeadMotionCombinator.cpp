#include "HeadMotionCombinator.h"
#include "Tools/Module/ModuleManager.h"

void HeadMotionCombinator::update()
{
    UPDATE_REPRESENTATION(HeadMotionEngineOutput);
}

void HeadMotionCombinator::update(HeadJointRequest &jointRequest)
{
    update();
    jointRequest.angles[Joints::headYaw] = theHeadMotionEngineOutput->pan;
    jointRequest.angles[Joints::headPitch] = theHeadMotionEngineOutput->tilt;
}
