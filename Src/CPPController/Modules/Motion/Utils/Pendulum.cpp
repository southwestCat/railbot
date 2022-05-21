#include "Pendulum.h"
#include "Modules/Motion/MotionConfigure.h"

void Pendulum::reset(const Vector3f &com, const Vector3f &comd, const Vector3f &comdd)
{
    comHeight_ = com.z();
    assert(comHeight_ > 0);
    float DEFAULT_HEIGHT = comHeight_;
    float DEFAULT_LAMDA = Constants::g / DEFAULT_HEIGHT;
    com_ = com;
    comd_ = comd;
    comdd_ = comdd;
    comddd_ = Vector3f::Zero();
    omega_ = sqrt(DEFAULT_LAMDA);
    zmp_ = com_ + (Constants::gravity - comdd_) / DEFAULT_LAMDA;
    zmpd_ = comd_ - comddd_ / DEFAULT_LAMDA;
}

void Pendulum::integrateIPM(Vector3f zmp, float lambda, float dt)
{
    Vector3f com_prev = com_;
    Vector3f comd_prev = comd_;
    omega_ = sqrt(lambda);
    zmp_ = zmp;

    //! Virtual repellent point
    Vector3f vrp = zmp_ - Constants::gravity / lambda;
    float ch = cosh(omega_ * dt);
    float sh = sinh(omega_ * dt);
    comdd_ = lambda * (com_prev - zmp_) + Constants::gravity;
    comd_ = comd_prev * ch + omega_ * (com_prev - vrp) * sh;
    com_ = com_prev * ch + comd_prev * sh / omega_ - vrp * (ch - 1.f);

    // default values for third-order terms
    comddd_ = Vector3f::Zero();
    zmpd_ = comd_ - comddd_ / lambda;
}

void Pendulum::integrateCoMJerk(const Vector3f &comddd, float dt)
{
    com_ += dt * (comd_ + dt * (comdd_ / 2 + dt * (comddd / 6)));
    comd_ += dt * (comdd_ + dt * (comddd / 2));
    comdd_ += dt * comddd;
    comddd_ = comddd;
}

void Pendulum::resetCoMHeight(float height)
{
    Vector3f n = Constants::e_z;
    com_ += (height + n.dot(-com_)) * n;
    comd_ -= n.dot(comd_) * n;
    comdd_ -= n.dot(comdd_) * n;
    comddd_ -= n.dot(comddd_) * n;
}

void Pendulum::completeIPM()
{
    Vector3f n = Constants::e_z;
    Vector3f gravitoInertial = Constants::gravity - comdd_;
    float lambda = n.dot(gravitoInertial) / n.dot(-com_);
    zmp_ = com_ + gravitoInertial / lambda;
    zmpd_ = comd_ - comddd_ / lambda;
    omega_ = sqrt(lambda);
}
