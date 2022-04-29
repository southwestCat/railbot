#pragma once

#include "Tools/Math/Eigen.h"
#include "RBInertia.h"

namespace sva
{
    /**
     * @brief Spatial Articulated Body Inertia compact representation.
     */
    class ABInertia
    {
    public:
        ABInertia() : M_(), H_(), I_() {}
        /**
         * @brief Construct a new ABInertia object
         *
         * @param M Mass matrix.
         * @param H Generalized inertia matrix.
         * @param I Inertia matrix.
         */
        ABInertia(const Matrix3f &M, const Matrix3f &H, const Matrix3f &I)
            : M_(Matrix3f::Zero()), H_(H), I_(Matrix3f::Zero())
        {
            M_.triangularView<Eigen::Lower>() = M;
            I_.triangularView<Eigen::Lower>() = I;
        }
        /**
         * @brief Construct a new ABInertia object
         *
         * @param ltM Lower triangular view of Mass matrix.
         * @param H Generalized inertia matrix.
         * @param ltI Lower triangular view Inertia matrix.
         */
        ABInertia(const Eigen::TriangularView<Matrix3f, Eigen::Lower> &ltM,
                  const Matrix3f &H,
                  const Eigen::TriangularView<Matrix3f, Eigen::Lower> &ltI)
            : M_(ltM), H_(H), I_(ltI)
        {
        }

        // Accessor
        const Matrix3f &lowerTriangularMassMatrix() const
        {
            return M_;
        }

        /// @return Mass matrix.
        Matrix3f massMatrix() const
        {
            Matrix3f M;
            M.template triangularView<Eigen::Upper>() = M_.transpose();
            M.template triangularView<Eigen::StrictlyLower>() = M_;
            return M;
        }

        /// @return Generalized inertia matrix.
        const Matrix3f &gInertia() const
        {
            return H_;
        }

        /// @return Inertia matrix with a zero upper part.
        const Matrix3f &lowerTriangularInertia() const
        {
            return I_;
        }

        /// @return Inertia matrix.
        Matrix3f inertia() const
        {
            Matrix3f I;
            I.template triangularView<Eigen::Upper>() = I_.transpose();
            I.template triangularView<Eigen::StrictlyLower>() = I_;
            return I;
        }

        /// @retrun Non compact spatial articulated body inertia matrix.
        Matrix6f matrix() const
        {
            Matrix6f m;
            m << inertia(), H_, H_.transpose(), massMatrix();
            return m;
        }

        // Operators
        ABInertia operator+(const ABInertia &rbI) const
        {
            Matrix3f M, I;
            M.template triangularView<Eigen::Lower>() = M_ + rbI.M_;
            I.template triangularView<Eigen::Lower>() = I_ + rbI.I_;
            return ABInertia(M, H_ + rbI.H_, I);
        }

        ABInertia operator-(const ABInertia &rbI) const
        {
            Matrix3f M, I;
            M.template triangularView<Eigen::Lower>() = M_ - rbI.M_;
            I.template triangularView<Eigen::Lower>() = I_ - rbI.I_;
            return ABInertia(M, H_ - rbI.H_, I);
        }

        ABInertia operator-() const
        {
            return ABInertia(-M_, -H_, -I_);
        }

        ABInertia &operator+=(const ABInertia &rbI)
        {
            M_.template triangularView<Eigen::Lower>() += rbI.M_;
            H_ += rbI.H_;
            I_.template triangularView<Eigen::Lower>() += rbI.I_;
            return *this;
        }

        ABInertia &operator-=(const ABInertia &rbI)
        {
            M_.template triangularView<Eigen::Lower>() -= rbI.M_;
            H_ -= rbI.H_;
            I_.template triangularView<Eigen::Lower>() -= rbI.I_;
            return *this;
        }

        ABInertia operator*(float scalar) const
        {
            Matrix3f M, I;
            M.triangularView<Eigen::Lower>() = scalar * M_;
            I.triangularView<Eigen::Lower>() = scalar * I_;
            return ABInertia(M, scalar * H_, I);
        }

        ABInertia operator+(const RBInertia &rbI) const
        {
            Matrix3f M, I;
            M.triangularView<Eigen::Lower>() = massMatrix() + Matrix3f::Identity() * rbI.mass();
            I.triangularView<Eigen::Lower>() = inertia() + rbI.inertia();
            return ABInertia(M, gInertia() + vector3ToCrossMatrix(rbI.momentum()), I);
        }

        ForceVec operator*(const MotionVec &mv) const
        {
            return ForceVec(inertia() * mv.angular() + gInertia() * mv.linear(),
                            massMatrix() * mv.linear() + gInertia().transpose() * mv.angular());
        }

        friend ABInertia operator*(float scalar, const ABInertia &abI)
        {
            return abI * scalar;
        }

        friend std::ostream &operator<<(std::ostream &o, const ABInertia &abI)
        {
            o << abI.matrix();
            return o;
        }

        template <typename Derived>
        void mul(const Eigen::MatrixBase<Derived> &mv, Eigen::MatrixBase<Derived> &result) const
        {
            forceCouple(result).noalias() = inertia() * motionAngular(mv);
            forceCouple(result).noalias() += gInertia() * motionLinear(mv);

            forceForce(result).noalias() = inertia() * motionLinear(mv);
            forceForce(result).noalias() += gInertia().transpose() * motionAngular(mv);
        }

        bool operator==(const ABInertia &abI) const
        {
            return M_ == abI.M_ && H_ == abI.H_ && I_ == abI.I_;
        }

        bool operator!=(const ABInertia &abI) const
        {
            return M_ != abI.M_ || H_ != abI.H_ || I_ != abI.I_;
        }

    private:
        Matrix3f M_;
        Matrix3f H_;
        Matrix3f I_;
    };
}