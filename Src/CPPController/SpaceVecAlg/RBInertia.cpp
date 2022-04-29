#include "RBInertia.h"

namespace sva
{
    Matrix3f inertiaToOrigin(const Matrix3f &inertia, float mass, const Vector3f &com, const Matrix3f &rotation)
    {
        Matrix3f trans =
            Eigen::vector3ToCrossMatrix(mass * com) * Eigen::vector3ToCrossMatrix(com).transpose();
        return rotation * (inertia + trans) * rotation.transpose();
    }
}