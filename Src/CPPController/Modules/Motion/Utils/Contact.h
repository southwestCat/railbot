#pragma once

#include "SpaceVecAlg/PTransform.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"

class Contact
{
public:
    Contact() = default;

public:
    enum class ContactState
    {
        DoubleSupport,
        LeftSupport,
        RightSupport
    };

    enum class SurfaceType
    {
        defaultContact,
        LeftFootContact,
        RightFootContact
    };

public:
    sva::PTransform anklePose;
    float halfLength;
    float halfWidth;
    sva::PTransform pose;
    Pose3f solePose;
    SurfaceType SurfaceType;

public:
    const sva::PTransform &poseW() const { return pose; }
    Vector3f sagital() const { return pose.rotation().row(0); }
    Vector3f lateral() const { return pose.rotation().row(1); }
    Vector3f normal() const { return pose.rotation().row(2); }
    const Vector3f &position() const { return pose.translation(); }
    const Vector3f &b() const { return lateral(); }
    const Vector3f &n() const { return normal(); }
    const Vector3f &t() const { return sagital(); }
    const Vector3f &p() const { return position(); }
    float x() const { return position()(0); }
    float y() const { return position()(1); }
    float z() const { return position()(2); }

    /** Corner vertex of the contact area.
     *
     */
    Vector3f vertex0() const
    {
        return position() + halfLength * t() + halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Vector3f vertex1() const
    {
        return position() + halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Vector3f vertex2() const
    {
        return position() - halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Vector3f vertex3() const
    {
        return position() - halfLength * t() + halfWidth * b();
    }

    template <int i>
    float minCoord() const
    {
        return std::min(std::min(vertex0()(i), vertex1()(i)), std::min(vertex2()(i), vertex3()(i)));
    }

    template <int i>
    float maxCoord() const
    {
        return std::max(std::max(vertex0()(i), vertex1()(i)), std::max(vertex2()(i), vertex3()(i)));
    }

    float xmin() const { return minCoord<0>(); }
    float xmax() const { return maxCoord<0>(); }
    float ymin() const { return minCoord<1>(); }
    float ymax() const { return maxCoord<1>(); }
    float zmin() const { return minCoord<2>(); }
    float zmax() const { return maxCoord<2>(); }

    /**
     * @brief Halfspace representation of contact area in the contact frame.
     *
     * @return HrepXf
     */
    HrepXf localHrep() const
    {
        Matrix4x2f localHrepMat;
        Vector4f localHrepVec;
        localHrepMat << +1, 0,
            -1, 0,
            0, +1,
            0, -1;
        localHrepVec << halfLength,
            halfLength,
            halfWidth,
            halfWidth;
        return HrepXf(localHrepMat, localHrepVec);
    }

    /**
     * @brief Halfspace representation of contact area in the world frame.
     *
     * @return HrepXf
     */
    HrepXf hrep() const
    {
        Matrix4x2f worldHrepMat;
        Vector4f worldHrepVec;
        HrepXf local = localHrep();
        auto &localHrepMat = local.first;
        auto &localHrepVec = local.second;
        const sva::PTransform &X_0_c = pose;
        worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
        worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
        return HrepXf(worldHrepMat, worldHrepVec);
    }

    /**
     * @brief Move contact by a given magnitude in a random direction.
     *
     * @param magnitude Absolute displacement after noising.
     */
    Contact addNoise(float magnitude) const
    {
        Contact noisedContact = *this;
        Vector3f unitRandom = Vector3f::Random().normalized();
        Vector3f displacement = magnitude * unitRandom;
        noisedContact.pose = sva::PTransform(displacement) * this->pose;
        return noisedContact;
    }

    friend Contact operator*(const sva::PTransform &X, const Contact &contact)
    {
        Contact result = contact;
        result.pose = X * contact.pose;
        return result;
    }
};
