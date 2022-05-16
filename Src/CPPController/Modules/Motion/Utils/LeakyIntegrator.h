#pragma once

template <typename T>
class LeakyIntegratorBase
{
public:
    void add(const T &value, float dt)
    {
        integral_ = (1. - rate_ * dt) * integral_ + dt * value;
        if (saturation_ > 0.f)
        {
            saturate();
        }
    }

    const T &eval() const { return integral_; }

    float rate() const { return rate_; }

    void rate(float r) { rate_ = r; }

    void saturation(float s) { saturation_ = s; }

protected:
    virtual void saturate() = 0;

protected:
    T integral_;
    float rate_ = 0.f;
    float saturation_ = -1.f;
};

template <typename T>
class LeakyIntegrator : public LeakyIntegratorBase<T>
{
public:
    LeakyIntegrator()
    {
        this->integral_.setZero();
    }

    virtual void saturate() override
    {
        for (unsigned i = 0; i < 3; i++)
        {
            if (this->integral_(i) < -this->saturation_)
            {
                this->integral_(i) = -this->saturation_;
            }
            else if (this->integral_(i) > this->saturation_)
            {
                this->integral_(i) = this->saturation_;
            }
        }
    }

    void setZero()
    {
        this->integral_.setZero();
    }
};

template <>
class LeakyIntegrator<float> : public LeakyIntegratorBase<float>
{
    LeakyIntegrator()
    {
        this->integral_ = 0.f;
    }

    virtual void saturate() override
    {
        if (this->integral_ < -this->saturation_)
        {
            this->integral_ = -this->saturation_;
        }
        else if (this->integral_ > this->saturation_)
        {
            this->integral_ = this->saturation_;
        }
    }

    void clamp(float min, float max)
    {
        if (this->integral_ < min)
        {
            this->integral_ = min;
        }
        else if (this->integral_ > max)
        {
            this->integral_ = max;
        }
    }

    void setZero()
    {
        this->integral_ = 0.f;
    }
};