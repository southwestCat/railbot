#pragma once

#include "Tools/Math/Constants.h"

template <typename T>
class LowPassVelocityFilter
{
public:
    LowPassVelocityFilter() = default;
    LowPassVelocityFilter(double dt) : dt_(dt) { reset(T::Zero()); }
    LowPassVelocityFilter(double dt, double period) : dt_(dt)
    {
        reset(T::Zero());
        cutoffPeriod(period);
    }
    double dt() const { return dt_; }
    void dt(double dt) { dt_ = dt; }
    double cutoffPeriod() const { return cutoffPeriod_; }
    void cutoffPeriod(double period) { cutoffPeriod_ = period; }

    /**
     * @brief Reset position to an initial rest value.
     *
     * @param pos
     */
    void reset(T pos)
    {
        pos_ = pos;
        vel_ = T::Zeros();
    }

    /**
     * @brief Update velocity estimate from new position value.
     *
     * @param newPos New observed position.
     */
    void update(const T &newPos)
    {
        float x = (cutoffPeriod_ <= dt_) ? 1.f : dt_ / cutoffPeriod_;
        T discVel = (newPos - pos_) / dt_;
        T newVel = x * discVel + (1.f - x) * vel_;
        pos_ = newPos;
        // vel_ = newVel;
        updateRingBuffer(newVel);
    }

    void updateRingBuffer(const T &v)
    {
        ringBuffer[p] = v;
        p++;
        if (p % 4 == 0)
        {
            p = 0;
        }
        vel_ = (ringBuffer[0] + ringBuffer[1] + ringBuffer[2] + ringBuffer[3]) / 4.0;
    }

    void updatePositionOnly(const T &newPos)
    {
        pos_ = newPos;
    }

    const T &vel() { return vel_; }

private:
    T pos_;
    T vel_;
    T ringBuffer[4];
    int p = 0;
    float cutoffPeriod_ = 0.f;
    const float dt_ = Constants::motionCycleTime;
};