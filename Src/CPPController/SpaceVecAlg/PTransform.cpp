#include "PTransform.h"
#include "MathFunc.h"
#include "Operator.h"
#include <algorithm>

using namespace std;

namespace sva
{
    Matrix3f RotX(float theta)
    {
        float s = sin(theta);
        float c = cos(theta);
        return (Matrix3f() << 1., 0., 0., 0., c, -s, 0., s, c).finished();
    }

    Matrix3f RotY(float theta)
    {
        float s = sin(theta);
        float c = cos(theta);
        return (Matrix3f() << c, 0., s, 0., 1., 0., -s, 0., c).finished();
    }

    Matrix3f RotZ(float theta)
    {
        float s = sin(theta);
        float c = cos(theta);
        return (Matrix3f() << c, -s, 0., s, c, 0., 0., 0., 1.).finished();
    }

    Vector3f rotationError(const Matrix3f &E_a_b, const Matrix3f &E_a_c)
    {
        Matrix3f E_b_c = E_a_c * E_a_b.transpose();
        return Vector3f(E_a_b.transpose() * rotationVelocity(E_b_c));
    }

    Vector3f rotationVelocity(const Matrix3f &E_a_b)
    {
        Vector3f w;
        float acosV = (E_a_b(0, 0) + E_a_b(1, 1) + E_a_b(2, 2) - 1.) * 0.5;
        float theta = acos(min(max(acosV, -1.f), 1.f));
        w = Vector3f(-E_a_b(2, 1) + E_a_b(1, 2), -E_a_b(0, 2) + E_a_b(2, 0), -E_a_b(1, 0) + E_a_b(0, 1));
        w *= sinc_inv(theta) * 0.5;

        return w;
    }

    MotionVec transformError(const PTransform &X_a_b, const PTransform &X_a_c)
    {
        PTransform X_b_c = X_a_c * X_a_b.inv();
        return PTransform(Matrix3f(X_a_b.rotation().transpose())) * transformVelocity(X_b_c);
    }

    MotionVec transformVelocity(const PTransform &X_a_b)
    {
        return MotionVec(rotationVelocity(X_a_b.rotation()), X_a_b.translation());
    }

    PTransform interpolate(const PTransform &from, const PTransform &to, float t)
    {
        Quaternionf qfrom(from.rotation());
        Quaternionf qto(to.rotation());
        PTransform result(qfrom.slerp(t, qto), (from.translation() * (1.f - t) + to.translation() * t));
        return result;
    }
}