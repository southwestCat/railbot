#pragma once

#include "Tools/Motion/SensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/RobotParts/Joints.h"

#include <array>
#include <cassert>

class JointAngles
{
public:
    enum
    {
        off = SensorData::off
    };
    enum
    {
        ignore = 20000
    };

    JointAngles();

    Angle mirror(Joints::Joint joint) const;

    void mirror(const JointAngles &other);
private:
    static float mirror(float angle);
public:
    std::array<Angle, Joints::numOfJoints> angles;
    unsigned timestamp = 0;
};

inline JointAngles::JointAngles()
{
    angles.fill(0.f);
}

inline void JointAngles::mirror(const JointAngles &other)
{
    assert(this != &other);
    for (int i = 0; i < Joints::numOfJoints; i++)
    {
        angles[i] = other.mirror(static_cast<Joints::Joint>(i));
    }
    timestamp = other.timestamp;
}

inline float JointAngles::mirror(float angle)
{
    return (angle == off || angle == ignore) ? angle : -angle;
}