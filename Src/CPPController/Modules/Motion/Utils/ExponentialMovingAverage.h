#pragma once

#include "Tools/Math/Eigen.h"

class ExponentialMovingAverage
{
public:
    ExponentialMovingAverage(float timeConstant, const Vector3f &initValue = Vector3f::Zero())
    {
        average_ = initValue;
        rawValue_ = initValue;
        this->timeConstant(timeConstant);
    }

    void append(const Vector3f &value)
    {
        average_ += alpha_ * (value - average_);
        rawValue_ = value;
        if (saturation_ > 0.f)
        {
            saturate();
        }
    }

    const Vector3f &eval() const
    {
        return average_;
    }

    const Vector3f &raw() const
    {
        return rawValue_;
    }

    void reset()
    {
        return setZero();
    }

    void saturation(float limit)
    {
        saturation_ = limit;
    }

    void setZero()
    {
        average_.setZero();
    }

    const float timeConstant() const
    {
        return timeConstant_;
    }

    void timeConstant(float T)
    {
        T = std::max(T, 2 * dt_);
        alpha_ = 1.f - std::exp(-dt_ / T);
        timeConstant_ = T;
    }

private:
    void saturate()
    {
        for (unsigned i = 0; i < 3; i++)
        {
            if (average_(i) < -saturation_)
            {
                average_(i) = -saturation_;
            }
            else if (average_(i) > saturation_)
            {
                average_(i) = saturation_;
            }
        }
    }

private:
    const float dt_ = Constants::motionCycleTime;
    float alpha_;
    float timeConstant_;
    float saturation_ = -1.f;
    Vector3f average_;
    Vector3f rawValue_;
};