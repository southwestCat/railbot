#pragma once

#include "Tools/Math/Eigen.h"
#include "ForceVec.h"

namespace sva
{

    class MotionVec
    {
    public:
        MotionVec() = default;
        /**
         * @brief Construct a new Motion Vec object
         *
         * @param vec
         */
        MotionVec(const Vector6f &vec) : angular_(vec.head(3)), linear_(vec.tail(3)) {}
        MotionVec(const Vector3f &angular, const Vector3f &linear)
            : angular_(angular), linear_(linear) {}

        static MotionVec zero()
        {
            return {Vector3f::Zero(), Vector3f::Zero()};
        }

        //! Accessor
        /**
         * @brief Get linear
         *
         * @return Vector3f&
         */
        Vector3f &linear() { return linear_; }
        const Vector3f &linear() const { return linear_; }

        /**
         * @brief Get angular
         *
         * @return Vector3f&
         */
        Vector3f &angular() { return angular_; }
        const Vector3f &angular() const { return angular_; }

        /**
         * @brief Non compact spatial motion vector.
         *
         * @return Vector6f
         */
        Vector6f vector() const
        {
            return (Vector6f() << angular_, linear_).finished();
        }

        //! operators
        MotionVec operator+(const MotionVec &mv) const
        {
            return MotionVec(angular_ + mv.angular(), linear_ + mv.linear());
        }

        MotionVec operator-(const MotionVec &mv) const
        {
            return MotionVec(angular_ - mv.angular(), linear_ - mv.linear());
        }

        MotionVec operator+=(const MotionVec &mv)
        {
            angular_ += mv.angular();
            linear_ += mv.linear();
            return *this;
        }

        MotionVec operator-=(const MotionVec &mv)
        {
            angular_ -= mv.angular();
            linear_ -= mv.linear();
            return *this;
        }

        MotionVec operator*(float scalar) const
        {
            return MotionVec(scalar * angular_, scalar * linear_);
        }

        friend MotionVec operator*(float scalar, const MotionVec &mv)
        {
            return mv * scalar;
        }

        MotionVec operator*=(float scalar)
        {
            angular_ *= scalar;
            linear_ *= scalar;
            return *this;
        }

        MotionVec operator/=(float scalar)
        {
            angular_ /= scalar;
            linear_ /= scalar;
            return *this;
        }

        MotionVec operator/(float scalar) const
        {
            return MotionVec(angular_ / scalar, linear_ / scalar);
        }

        MotionVec cross(const MotionVec &mv) const
        {
            return MotionVec(angular_.cross(mv.angular()), angular_.cross(mv.linear()) + linear_.cross(mv.angular()));
        }

        ForceVec crossDual(const ForceVec &fv) const
        {
            return ForceVec(angular().cross(fv.couple()) + linear().cross(fv.force()), angular().cross(fv.force()));
        }

        float dot(ForceVec fv) const
        {
            return angular().dot(fv.couple()) + linear().dot(fv.force());
        }

        bool operator==(const MotionVec &mv) const
        {
            return (angular_ == mv.angular()) && (linear_ == mv.linear());
        }

        bool operator!=(const MotionVec &mv) const
        {
            return (angular_ != mv.angular()) || (linear_ != mv.linear());
        }

        friend std::ostream &operator<<(std::ostream &o, const MotionVec &mv)
        {
            o << mv.vector().transpose();
            return o;
        }

    private:
        Vector3f angular_;
        Vector3f linear_;
    };
}