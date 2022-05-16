#pragma once

#include "Tools/Math/Eigen.h"

class Pendulum
{
public:
    Pendulum(const Vector3f &com = Vector3f::Zero(), const Vector3f &comd = Vector3f::Zero(), const Vector3f &comdd = Vector3f::Zero()) {}
    void completeIPM();
    void integrateCoMJerk(const Vector3f &comddd, float dt);
    void integrateIPM(Vector3f zmp, float lambda, float dt = 0.01);
    void reset(const Vector3f &com, const Vector3f &comd, const Vector3f &comdd);
    void resetCoMHeight(float height);
    const Vector3f &com() const { return com_; }
    const Vector3f &comd() const { return comd_; }
    const Vector3f &comdd() const { return comdd_; }
    Vector3f dcm() const { return com_ + comd_ / omega_; }
    float omega() const { return omega_; }
    const Vector3f &zmp() const { return zmp_; }
    const Vector3f &zmpd() const { return zmpd_; }
    const bool &needReset() const { return needReset_; }
    bool &needReset() { return needReset_; }
    const float &comHeight() const { return comHeight_; }
    float &comHeight() { return comHeight_; }

private:
    const float dt = Constants::motionCycleTime;
    Vector3f com_;                                    //< Position of the center of mass.
    Vector3f comd_;                                   //< Velocity of the center of mass.
    Vector3f comdd_;                                  //< Acceleration of the center of mass.
    Vector3f comddd_;                                 //< Jerk of the center of mass.
    Vector3f zmp_;                                    //< Position of the zero-tilting moment point.
    Vector3f zmpd_;                                   //< Velocity of the zero-tilting moment point.
    float omega_;                                     //< Natural frequency of the linear inverted pendulum.
    bool needReset_ = true;
    float comHeight_ = -1.f;
};