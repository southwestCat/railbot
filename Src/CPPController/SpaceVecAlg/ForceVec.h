#pragma once

#include "Tools/Math/Eigen.h"
#include <iostream>

namespace sva
{

    class ForceVec
    {
    public:
        ForceVec() : couple_(), force_() {}
        /**
         * @brief Construct a new Force Vec object
         *
         * @param vec Spatial force vector with couple in head and force in tail.
         */
        ForceVec(const Vector6f &vec) : couple_(vec.head(3)), force_(vec.tail(3)) {}

        /**
         * @brief Construct a new Force Vec object
         *
         * @param couple
         * @param force
         */
        ForceVec(const Vector3f &couple, const Vector3f &force)
            : couple_(couple), force_(force) {}

        //! Accessor

        /**
         * @brief couple is used in SpaceVecAlg with the general meaning of mement.
         *
         * @return Vector3f&
         */
        Vector3f &couple() { return couple_; }
        const Vector3f &couple() const { return couple_; }
        Vector3f &moment() { return couple_; }
        const Vector3f &moment() const { return couple_; }

        /**
         * @brief Get force
         *
         * @return Vector3f&
         */
        Vector3f &force() { return force_; }
        const Vector3f &force() const { return force_; }

        Vector6f vector() const
        {
            return (Vector6f() << couple_, force_).finished();
        }

        ForceVec operator+(const ForceVec &fv) const
        {
            return ForceVec(couple_ + fv.couple(), force_ + fv.force());
        }

        ForceVec operator-(const ForceVec &fv) const
        {
            return ForceVec(couple_ - fv.couple(), force_ - fv.force());
        }

        ForceVec operator-() const
        {
            return ForceVec(-couple_, -force_);
        }

        ForceVec &operator+=(const ForceVec &fv)
        {
            couple_ += fv.couple();
            force_ += fv.force();
            return *this;
        }

        ForceVec &operator-=(const ForceVec &fv)
        {
            couple_ -= fv.couple();
            force_ -= fv.force();
            return *this;
        }

        ForceVec operator*(float scalar) const
        {
            return ForceVec(scalar * couple_, scalar * force_);
        }

        friend ForceVec operator*(float scalar, const ForceVec &fv)
        {
            return fv * scalar;
        }

        ForceVec operator/(float scalar) const
        {
            return ForceVec(couple_ / scalar, force_ / scalar);
        }

        ForceVec &operator*=(float scalar)
        {
            couple_ *= scalar;
            force_ *= scalar;
            return *this;
        }

        ForceVec &operator/=(float scalar)
        {
            couple_ /= scalar;
            force_ /= scalar;
            return *this;
        }

        bool operator==(const ForceVec &fv) const
        {
            return (couple_ == fv.couple()) && (force_ == fv.force());
        }

        bool operator!=(const ForceVec &fv) const
        {
            return (couple_ != fv.couple()) || (force_ != fv.force());
        }

        friend std::ostream &operator<<(std::ostream &o, const ForceVec &fv)
        {
            o << fv.vector().transpose();
            return o;
        }

        static ForceVec
        Zero()
        {
            return ForceVec(Vector3f::Zero(), Vector3f::Zero());
        }

    private:
        Vector3f couple_;
        Vector3f force_;
    };

}