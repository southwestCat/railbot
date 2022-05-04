#include "HeadMotionEngine.h"
#include "Tools/Range.h"
#include "Representations/Infrastructure/JointAngles.h"

#include <algorithm>

HeadMotionEngine::HeadMotionEngine()
{
    requestedPan = requestedTilt = JointAngles::off;
    lastSpeed = Vector2f::Zero();
}

void HeadMotionEngine::update()
{
}

void HeadMotionEngine::update(HeadMotionEngineOutput &headMotionEngineOutput)
{
    requestedPan = theHeadMotionRequest->pan;
    requestedTilt = theHeadMotionRequest->tilt;

    constexpr float maxAcc = 10.f;
    const float pan = requestedPan == JointAngles::off ? JointAngles::off : Rangef(theHeadLimits->minPan(), theHeadLimits->maxPan()).limit(requestedPan);
    const float tilt = requestedTilt == JointAngles::off ? JointAngles::off : theHeadLimits->getTiltBound(pan).limit(requestedTilt);

    constexpr float deltaTime = Constants::motionCycleTime;
    const Vector2f position(headMotionEngineOutput.pan == JointAngles::off ? theJointSensorData->angles[Joints::headYaw] : headMotionEngineOutput.pan,
                            headMotionEngineOutput.tilt == JointAngles::off ? theJointSensorData->angles[Joints::headPitch] : headMotionEngineOutput.tilt);
    const Vector2f target(pan == JointAngles::off ? 0.f : pan, tilt == JointAngles::off ? 0.f : tilt);
    Vector2f offset(target - position);
    const float distanceToTarget = offset.norm();

    // calculate max speed
    const float maxSpeedForDistance = std::sqrt(2.f * distanceToTarget * maxAcc * 0.8f);

    const float requestedSpeed = static_cast<float>(theHeadMotionRequest->speed);

    const float maxSpeed = std::min(maxSpeedForDistance, requestedSpeed);

    // max speed clipping
    if (distanceToTarget / deltaTime > maxSpeed)
        offset *= maxSpeed * deltaTime / distanceToTarget; //<=> offset.normalize(maxSpeed * deltaTime);

    // max acceleration clipping
    Vector2f speed(offset / deltaTime);
    Vector2f acc((speed - lastSpeed) / deltaTime);
    const float accSquareAbs = acc.squaredNorm();
    if (accSquareAbs > maxAcc * maxAcc)
    {
        acc *= maxAcc * deltaTime / std::sqrt(accSquareAbs);
        speed = acc + lastSpeed;
        offset = speed * deltaTime;
    }
    /* <=>
  Vector2f speed(offset / deltaTime);
  Vector2f acc((speed - lastSpeed) / deltaTime);
  if(acc.squaredNorm() > maxAcc * maxAcc)
  {
    speed = acc.normalize(maxAcc * deltaTime) + lastSpeed;
    offset = speed * deltaTime;
  }
   */
    // PLOT("module:HeadMotionEngine:speed", toDegrees(speed.norm()));

    // calculate new position
    Vector2f newPosition(position + offset);

    // set new position
    headMotionEngineOutput.pan = pan == JointAngles::off ? JointAngles::off : newPosition.x();
    headMotionEngineOutput.tilt = tilt == JointAngles::off ? JointAngles::off : newPosition.y();
    headMotionEngineOutput.moving = pan != JointAngles::off && tilt != JointAngles::off && ((newPosition - position) / deltaTime).squaredNorm() > pow(maxAcc * deltaTime * 0.5f, 2);

    // check reachability
    headMotionEngineOutput.reachable = true;
    if (pan != requestedPan || tilt != requestedTilt)
        headMotionEngineOutput.reachable = false;

    // store some values for the next iteration
    lastSpeed = speed;
}
