#include "StandEngine.h"
#include "Tools/Motion/MotionUtilities.h"

StandEngine::StandEngine()
{
    MotionUtilities::stand(targetJoint);
}

void StandEngine::update()
{

}

void StandEngine::update(StandEngineOuptut &s)
{
    update();

    if (theMotionRequest->motion != MotionRequest::stand)
    {
        startTime = theFrameInfo->time;
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            startJoint.angles[i] = theJointAngles->angles[i];
        }
        return;
    }

    nowTime = theFrameInfo->getTimeSince(startTime);
    float ratio = (float)nowTime / (float)interpolateTime;
    
}