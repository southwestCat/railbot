#include "MotionCombinator.h"
#include "Tools/Module/ModuleManager.h"


void MotionCombinator::update()
{
    UPDATE_REPRESENTATION(HeadMotionEngineOutput);
}

void MotionCombinator::update(JointRequest &jointRequest)
{
    update();
}
