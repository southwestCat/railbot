#pragma once

#include "Tools/Math/Eigen.h"
#include "SpaceVecAlg/EigenUtility.h"

#include "MotionVec.h"
#include "Operator.h"
#include "RBInertia.h"
#include "ABInertia.h"

namespace sva
{
    class PTransform;
    /**
     * @brief Create a rotation matrix about the X axis.
     * The rotation is exprimed in successor frame
     *
     * @param theta rotation in radian. [rad]
     * @return Matrix3f
     */
    Matrix3f RotX(float theta);

    /**
     * @brief Create a rotation matrix about the Y axis.
     *
     * @param theta rotation in radian. [rad]
     * @return Matrix3f
     */
    Matrix3f RotY(float theta);

    /**
     * @brief Create a rotation matrix about the Z axis.
     *
     * @param theta rotation in radian. [rad]
     * @return Matrix3f
     */
    Matrix3f RotZ(float theta);

    /**
     * @brief Compute the 3D rotation error between two matrix E_a_b and E_a_c in the 'a' frame.
     * This method convert the 3D rotation matrix E_b_c into a rotation vector.
     * The matrix E_b_c is computed as follow E_a_c = E_b_c * E_a_b.
     * Then the error is computed with E_b_a*rotationVelocity(E_b_c).
     *
     * @param E_a_b
     * @param E_a_c
     * @return Vector3f
     */
    Vector3f rotationError(const Matrix3f &E_a_b, const Matrix3f &E_a_c);

    /**
     * @brief Compute the 3D rotation vector of the rotation matrix E_a_b in the 'a' frame.
     * If we integrate this rotation vector for 1 second we must
     * have the rotation matrix E_a_b.
     *
     * @param E_a_b
     * @return Vector3f
     */
    Vector3f rotationVelocity(const Matrix3f &E_a_b);

    /**
     * @brief Compute the 6D error between two PTransform in the 'a' frame.
     * This method convert the 6D transformation matrix X_b_c into a motion vector.
     * The matrix X_b_c is computed as follow X_a_c = X_b_c*X_a_b.
     * Then the error is computed with PTransform(E_b_a)*transformVelocity(X_b_c).
     *
     * @param X_a_b
     * @param X_a_c
     * @return MotionVec
     */
    MotionVec transformError(const PTransform &X_a_b, const PTransform &X_a_c);

    /**
     * @brief Compute the motion vector of the matrix X_a_b in the 'a' frame.
     * If we integrate this motion vector for 1 second we must
     * have the transformation matrix X_a_b.
     *
     * @param X_a_b
     * @return MotionVec
     */
    MotionVec transformVelocity(const PTransform &X_a_b);

    /**
     * @brief the spherical linear interpolation between the two quaternions
     * this and other at the parameter t in [0;1]. This represents an interpolation
     * for a constant motion between *this and other,
     * see also http://en.wikipedia.org/wiki/Slerp.
     *
     * @param from Start direction
     * @param to End direction
     * @param t Speed
     * @return PTransform
     */
    PTransform interpolate(const PTransform &from, const PTransform &to, float t);

    class PTransform
    {
    public:
        PTransform() : E_(), r_() {}
        /**
         * @brief Construct a new PTransform object
         *
         * @param pt
         */
        PTransform(const PTransform &pt) : E_(pt.rotation()), r_(pt.translation()) {}

        /**
         * @brief Construct a new PTransform object
         *
         * @param rot Rotation matrix.
         * @param trans Translation vector.
         */
        PTransform(const Matrix3f &rot, const Vector3f &trans) : E_(rot), r_(trans) {}

        /**
         * @brief Construct a new PTransform object
         *
         * @param rot Rotation quaternion.
         * @param trans Translation vector.
         */
        PTransform(const Quaternionf &rot, const Vector3f &trans) : E_(rot.matrix()), r_(trans) {}

        /**
         * @brief Rotation only transform.
         *
         * @param rot Rotation quaternion.
         */
        PTransform(const Quaternionf &rot) : E_(rot.matrix()), r_(Vector3f::Zero()) {}

        /**
         * @brief Rotation only transform.
         *
         * @param rot Rotation matrix.
         */
        PTransform(const Matrix3f &rot) : E_(rot), r_(Vector3f::Zero()) {}

        /**
         * @brief Translation only transform.
         *
         * @param trans Translation vector.
         */
        PTransform(const Vector3f &trans) : E_(Matrix3f::Identity()), r_(trans) {}

        const Matrix3f &rotation() const { return E_; }
        Matrix3f &rotation() { return E_; }

        const Vector3f &translation() const { return r_; }
        Vector3f &translation() { return r_; }

        sva::PTransform toMeter()
        {
            float x = r_.x() / 1000.f;
            float y = r_.y() / 1000.f;
            float z = r_.z() / 1000.f;
            return {E_, {x, y, z}};
        }

        Matrix6f matrix() const
        {
            Matrix6f m;
            m << E_, Matrix3f::Zero(), vector3ToCrossMatrix(r_) * E_, E_;
            return m;
        }

        Matrix6f dualMatrix() const
        {
            Matrix6f m;
            m << E_, vector3ToCrossMatrix(r_) * E_, Matrix3f::Zero(), E_;
            return m;
        }

        Vector3f angularMul(const MotionVec &mv) const
        {
            const Matrix3f &E = rotation();
            return (E * mv.angular()).eval();
        }

        Vector3f linearMul(const MotionVec &mv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            return (E * (mv.linear() - r.cross(mv.angular()))).eval();
        }

        //! Operators

        PTransform operator*(const PTransform &pt) const
        {
            return PTransform(E_ * pt.E_, pt.r_ + pt.E_.transpose() * r_);
        }

        MotionVec operator*(const MotionVec &mv) const
        {
            return MotionVec(angularMul(mv), linearMul(mv));
        }

        template <typename Derived>
        void mul(const Eigen::MatrixBase<Derived> &mv, Eigen::MatrixBase<Derived> &result) const
        {
            static_assert(Derived::RowsAtCompileTime == 6, "the matrix must have exactly 6 rows");
            static_assert(std::is_same<typename Derived::Scalar, float>::value, "motion vec and matrix must be the same type");

            const Matrix3f &E = rotation();
            const Vector3f &r = translation();

            motionAngular(result).noalias() = E * motionAngular(mv);

            motionLinear(result).noalias() = motionLinear(mv);
            sva_internal::colwiseCrossPlusEq(motionAngular(mv), r, motionLinear(result));
            sva_internal::colwiseLeftMultEq(motionLinear(result), E, motionLinear(result));
        }

        MotionVec invMul(const MotionVec &mv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            MotionVec ret;
            ret.angular().noalias() = E.transpose() * mv.angular();
            ret.linear().noalias() = E.transpose() * mv.linear();
            ret.linear().noalias() += r.cross(ret.angular());
            return ret;
        }

        Vector3f angularInvMul(const MotionVec &mv) const
        {
            const Matrix3f &E = rotation();
            return (E.transpose() * mv.angular()).eval();
        }

        Vector3f linearInvMul(const MotionVec &mv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            return (E.transpose() * mv.linear() + r.cross(E.transpose() * mv.angular())).eval();
        }

        template <typename Derived>
        void invMul(const Eigen::MatrixBase<Derived> &mv, Eigen::MatrixBase<Derived> &result) const
        {
            static_assert(Derived::RowsAtCompileTime == 6, "the matrix must have exactly 6 rows");
            static_assert(std::is_same<typename Derived::Scalar, float>::value, "motion vec and matrix must be the same type");

            const Eigen::Matrix3f &E = rotation();
            const Eigen::Vector3f &r = translation();

            motionAngular(result).noalias() = E.transpose() * motionAngular(mv);

            motionLinear(result).noalias() = E.transpose() * motionLinear(mv);
            sva_internal::colwiseCrossMinusEq(motionAngular(result), r, motionLinear(result));
        }

        Vector3f coupleDualMul(const ForceVec &fv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            // return (E * (fv.couple() - r.cross(fv.force()))).eval();
            return (E*fv.couple() + r.cross(E*fv.force())).eval();
        }

        Vector3f forceDualMul(const ForceVec &fv) const
        {
            const Matrix3f &E = rotation();
            return (E * fv.force()).eval();
        }

        ForceVec dualMul(const ForceVec &fv) const
        {
            return ForceVec(coupleDualMul(fv), forceDualMul(fv));
        }

        template <typename Derived>
        void dualMul(const Eigen::MatrixBase<Derived> &fv, Eigen::MatrixBase<Derived> &result) const
        {
            static_assert(Derived::RowsAtCompileTime == 6, "the matrix must have exactly 6 rows");
            static_assert(std::is_same<typename Derived::Scalar, float>::value, "force vec and matrix must be the same type");

            const Matrix3f &E = rotation();
            const Vector3f &r = translation();

            forceCouple(result).noalias() = forceCouple(fv);
            sva_internal::colwiseCrossPlusEq(forceForce(fv), r, forceCouple(result));
            sva_internal::colwiseLeftMultEq(forceCouple(result), E, forceCouple(result));

            forceForce(result).noalias() = E * forceForce(fv);
        }

        ForceVec transMul(const ForceVec &fv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            ForceVec ret;
            ret.force().noalias() = E.transpose() * fv.force();
            ret.couple().noalias() = E.transpose() * fv.couple();
            ret.couple().noalias() += r.cross(ret.force());
            return ret;
        }

        Vector3f coupleTransMul(const ForceVec &fv) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            return (E.transpose() * fv.couple() + r.cross(E.transpose() * fv.force())).eval();
        }

        Vector3f forceTransMul(const ForceVec &fv) const
        {
            const Matrix3f &E = rotation();
            return (E.transpose() * fv.force()).eval();
        }

        template <typename Derived>
        void transMul(const Eigen::MatrixBase<Derived> &fv, Eigen::MatrixBase<Derived> &result) const
        {
            static_assert(Derived::RowsAtCompileTime == 6, "the matrix must have exactly 6 rows");
            static_assert(std::is_same<typename Derived::Scalar, float>::value, "force vec and matrix must be the same type");

            const Eigen::Matrix3f &E = rotation();
            const Eigen::Vector3f &r = translation();

            forceForce(result).noalias() = E.transpose() * forceForce(fv);

            forceCouple(result).noalias() = E.transpose() * forceCouple(fv);
            sva_internal::colwiseCrossMinusEq(forceForce(result), r, forceCouple(result));
        }

        RBInertia dualMul(const RBInertia &rbI) const
        {
            const Matrix3f &E = rotation();
            const Vector3f &r = translation();
            Matrix3f I;
            I.triangularView<Eigen::Lower>() = E * (rbI.inertia() + vector3ToCrossMatrix(r) * Eigen::vector3ToCrossMatrix(rbI.momentum()) + Eigen::vector3ToCrossMatrix(rbI.momentum() - rbI.mass() * r) * Eigen::vector3ToCrossMatrix(r)) * E.transpose();
            return RBInertia(rbI.mass(), E * (rbI.momentum() - rbI.mass() * r), I);
        }

        RBInertia transMul(const RBInertia &rbI) const
        {
            const Eigen::Matrix3f &E = rotation();
            const Eigen::Vector3f &r = translation();
            Eigen::Matrix3f I;
            I.triangularView<Eigen::Lower>() =
                E.transpose() * rbI.inertia() * E - Eigen::vector3ToCrossMatrix(r) * Eigen::vector3ToCrossMatrix(E.transpose() * rbI.momentum()) - Eigen::vector3ToCrossMatrix(E.transpose() * rbI.momentum() + rbI.mass() * r) * Eigen::vector3ToCrossMatrix(r);
            return RBInertia(rbI.mass(), E.transpose() * rbI.momentum() + rbI.mass() * r, I);
        }

        ABInertia dualMul(const ABInertia &rbI) const
        {
            const Eigen::Matrix3f &E = rotation();
            const Eigen::Vector3f &r = translation();

            Eigen::Matrix3f massM = rbI.massMatrix();
            Eigen::Matrix3f rCross = vector3ToCrossMatrix(r);
            Eigen::Matrix3f tmpI = rbI.gInertia() - rCross * massM;

            Eigen::Matrix3f M, I;
            M.template triangularView<Eigen::Lower>() = E * massM * E.transpose();
            I.template triangularView<Eigen::Lower>() =
                E * (rbI.inertia() - rCross * rbI.gInertia().transpose() + (tmpI * rCross)) * E.transpose();

            return ABInertia(M, E * (tmpI)*E.transpose(), I);
        }

        ABInertia transMul(const ABInertia &rbI) const
        {
            const Eigen::Matrix3f &E = rotation();
            const Eigen::Vector3f &r = translation();

            Eigen::Matrix3f Mp(E.transpose() * rbI.massMatrix() * E);
            Eigen::Matrix3f Hp(E.transpose() * rbI.gInertia() * E);
            Eigen::Matrix3f rCross(vector3ToCrossMatrix(r));

            Eigen::Matrix3f M, I;
            M.triangularView<Eigen::Lower>() = Mp;
            I.triangularView<Eigen::Lower>() =
                (E.transpose() * rbI.inertia() * E + rCross * Hp.transpose() - (Hp + rCross * Mp) * rCross);
            return ABInertia(M, Hp + rCross * Mp, I);
        }

        /// @return Inverse Plücker transformation.
        PTransform inv() const
        {
            return PTransform(E_.transpose(), -E_.transpose() * r_);
        }

        bool operator==(const PTransform &pt) const
        {
            return E_ == pt.E_ && r_ == pt.r_;
        }

        bool operator!=(const PTransform &pt) const
        {
            return E_ != pt.E_ || r_ != pt.r_;
        }

        static PTransform Identity()
        {
            return PTransform(Matrix3f::Identity(), Vector3f::Zero());
        }

        friend std::ostream &operator<<(std::ostream &o, const PTransform &pt)
        {
            o << pt.matrix();
            return o;
        }

    private:
        Matrix3f E_;
        Vector3f r_;
    };
}