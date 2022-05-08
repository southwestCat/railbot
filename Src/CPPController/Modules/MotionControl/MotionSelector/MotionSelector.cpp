#include "MotionSelector.h"
#include "Tools/Module/ModuleManager.h"

void MotionSelector::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
}

void MotionSelector::update(LegMotionSelection &legMotionSelection)
{
    update();

    MotionRequest::Motion requestLegMotion = theMotionRequest->motion;

    legMotionSelection.targetMotion = requestLegMotion;
    
    int interpolationTime = interpolationTimes[legMotionSelection.targetMotion];

    if (legMotionSelection.targetMotion == MotionRequest::walk && lastLegMotion == MotionRequest::stand)
    {
        interpolationTime = 1;
    }

    int bodyInterpolationTime = interpolationTime;
    interpolate(legMotionSelection.ratios.data(), MotionRequest::numOfMotions, bodyInterpolationTime, legMotionSelection.targetMotion);

    lastLegMotion = legMotionSelection.targetMotion;
}   


void MotionSelector::interpolate(float *ratios, const int amount, const int interpolationTime, const int targetMotion)
{
    // increase / decrease all ratios according to target motion
    // const unsigned deltaTime = theFrameInfo.getTimeSince(lastExecution);
    const unsigned deltaTime = theFrameInfo->getTimeSince(std::min(lastExecutionLeg, lastExecutionArm));
    float delta = static_cast<float>(deltaTime) / interpolationTime;
    //ASSERT(SystemCall::getMode() == SystemCall::logFileReplay || delta > 0.00001f);
    float sum = 0;
    for (int i = 0; i < amount; i++)
    {
        if (i == targetMotion)
            ratios[i] += delta;
        else
            ratios[i] -= delta;
        ratios[i] = std::max(ratios[i], 0.0f); // clip ratios
        sum += ratios[i];
    }
    assert(sum != 0);
    // normalize ratios
    for (int i = 0; i < amount; i++)
    {
        ratios[i] /= sum;
        if (std::abs(ratios[i] - 1.f) < 0.00001f)
            ratios[i] = 1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
    }
}
