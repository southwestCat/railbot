#pragma once

#include "Tools/Math/Eigen.h"

namespace Eigen
{
    inline Matrix3f vector3ToCrossMatrix(const Vector3f &vec)
    {
        Matrix3f mat;
        mat << 0.f, -vec(2), vec(1),
            vec(2), 0.f, -vec(0),
            -vec(1), vec(0), 0.f;
        return mat;
    }

    inline Matrix6f vector6ToCrossMatrix(const Vector6f &vec)
    {
        Matrix6f mat;
        Matrix3f c13 = vector3ToCrossMatrix(vec.head(3));
        mat << c13, Matrix3f::Zero(), vector3ToCrossMatrix(vec.tail(3)), c13;
        return mat;
    }

    inline Matrix6f vector6ToCrossDualMatrix(const Vector6f &vec)
    {
        return -vector6ToCrossMatrix(vec).transpose();
    }

}

namespace sva
{
    using Eigen::vector3ToCrossMatrix;
    using Eigen::vector6ToCrossDualMatrix;
    using Eigen::vector6ToCrossMatrix;
}