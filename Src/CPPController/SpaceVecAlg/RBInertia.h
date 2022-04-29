#pragma once

#include "Tools/Math/Eigen.h"
#include "EigenUtility.h"
#include "MotionVec.h"
#include "ForceVec.h"
#include "Operator.h"

namespace sva
{
    Matrix3f inertiaToOrigin(const Matrix3f &inertia, float mass, const Vector3f &com, const Matrix3f &rotation);

    /**
     * @brief Spatial Rigid Boay Inertial compact representation.
     * "Rigid Body Dynamics Algorithms" page 245 A.3 Compact Representations
     */
    class RBInertia
    {
    public:
        RBInertia() : m_(), h_(), I_() {}
        /**
         * @brief Construct a new RBInertia object
         *
         * @param m Mass.
         * @param h Spatial momentum.
         * @param I Inertia matrix at body origin.
         */
        RBInertia(float m, const Vector3f &h, const Matrix3f &I)
            : m_(m), h_(h), I_(Matrix3f::Zero())
        {
            I_.triangularView<Eigen::Lower>() = I;
        }

        /**
         * @brief Construct a new RBInertia object
         *
         * @param m Mass.
         * @param h Spatial momentum.
         * @param ltI Eigen::Lower triangular view of Inertia matrix at body origin.
         */
        RBInertia(float m, const Vector3f &h, const Eigen::TriangularView<Matrix3f, Eigen::Lower> &ltI)
            : m_(m), h_(h), I_(ltI) {}

        //! Accessor
        float mass() const { return m_; }

        const Vector3f &momentum() const { return h_; }

        const Matrix3f &lowTriangularInertia() const { return I_; }

        Matrix3f inertia() const
        {
            Matrix3f I;
            I.triangularView<Eigen::Upper>() = I_.transpose();
            I.triangularView<Eigen::StrictlyLower>() = I_;
            return I;
        }

        Matrix6f matrix() const
        {
            Matrix6f m;
            Matrix3f hCross = vector3ToCrossMatrix(h_);
            m << inertia(), hCross, hCross.transpose(), Matrix3f::Identity() * m_;
            return m;
        }

        RBInertia operator+(const RBInertia &rbI) const
        {
            Matrix3f I;
            I.triangularView<Eigen::Lower>() = I_ + rbI.lowTriangularInertia();
            return RBInertia(m_ + rbI.mass(), h_ + rbI.momentum(), I);
        }

        RBInertia operator-(const RBInertia &rbI) const
        {
            Matrix3f I;
            I.triangularView<Eigen::Lower>() = I_ - rbI.I_;
            return RBInertia(m_ - rbI.m_, h_ - rbI.h_, I);
        }

        RBInertia operator-() const
        {
            return RBInertia(-m_, -h_, -I_);
        }

        RBInertia &operator+=(const RBInertia &rbI)
        {
            I_.triangularView<Eigen::Lower>() += rbI.I_;
            m_ += rbI.m_;
            h_ += rbI.h_;
            return *this;
        }

        RBInertia &operator-=(const RBInertia &rbI)
        {
            I_.triangularView<Eigen::Lower>() -= rbI.I_;
            m_ -= rbI.m_;
            h_ -= rbI.h_;
            return *this;
        }

        RBInertia operator*(float scalar) const
        {
            Matrix3f I;
            I.triangularView<Eigen::Lower>() = scalar * I_;
            return RBInertia(scalar * m_, scalar * h_, I);
        }

        ForceVec operator*(const MotionVec &mv) const
        {
            return ForceVec(inertia() * mv.angular() + momentum().cross(mv.linear()),
                            mass() * mv.linear() - momentum().cross(mv.angular()));
        }

        friend RBInertia operator*(float scalar, const RBInertia &rbI)
        {
            return rbI * scalar;
        }

        friend std::ostream &operator<<(std::ostream &o, const RBInertia &rbI)
        {
            o << rbI.matrix();
            return o;
        }

        template <typename Derived>
        void mul(const Eigen::MatrixBase<Derived> &mv, Eigen::MatrixBase<Derived> &result) const
        {
            forceCouple(result).noalias() = inertia() * motionAngular(mv);
            sva_internal::colwiseCrossMinusEq(motionLinear(mv), momentum(), forceCouple(result));

            forceForce(result).noalias() = motionLinear(mv) * mass();
            sva_internal::colwiseCrossPlusEq(motionAngular(mv), momentum(), forceForce(result));
        }

        bool operator==(const RBInertia &rbI) const
        {
            return m_ == rbI.m_ && h_ == rbI.h_ && I_ == rbI.I_;
        }

        bool operator!=(const RBInertia &rbI) const
        {
            return m_ != rbI.m_ || h_ != rbI.h_ || I_ != rbI.I_;
        }

    private:
        float m_;
        Vector3f h_;
        Matrix3f I_;
    };
}