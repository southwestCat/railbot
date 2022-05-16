#pragma once

#include "ExponentialMovingAverage.h"
#include "Tools/Math/Eigen.h"

class StationaryOffsetFilter
{
public:
    StationaryOffsetFilter(float timeConstant, const Vector3f &initValue = Vector3f::Zero())
        : average_(timeConstant, initValue)
    {
        filteredValue_ = initValue;
        rawValue_ = initValue;
    }

    void update(const Vector3f &value)
    {
        average_.append(value);
        filteredValue_ = value - average_.eval();
        rawValue_ = value;
    }

    const Vector3f &eval() const
    {
        return filteredValue_;
    }

    const Vector3f &raw() const
    {
        return rawValue_;
    }

    void setZero()
    {
        average_.setZero();
        filteredValue_.setZero();
        rawValue_.setZero();
    }

    const float timeConstant() const
    {
        return average_.timeConstant();
    }

    void timeConstant(float T)
    {
        average_.timeConstant(T);
    }

private:
    const float dt_ = Constants::motionCycleTime;
    Vector3f filteredValue_;
    Vector3f rawValue_;
    ExponentialMovingAverage average_;
};