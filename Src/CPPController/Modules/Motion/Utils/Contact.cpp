#include "Contact.h"
#include <cassert>

void Contact::calcPose(const RobotModel &robotModel, float halfLength, float halfWidth, SurfaceType surface, const sva::PTransform &WTO)
{
    halfLength_ = halfLength;
    halfWidth_ = halfWidth;
    surfaceType_ = surface;
    if (surface == SurfaceType::LeftFootContact)
    {
        const Pose3f p = robotModel.soleLeft;
        Pose3f c = p + Vector3f(30.f, 5.62f, 0.f);
        pose_ = WTO * sva::PTransform(c.rotation, c.translation);
    }
    else if (surface == SurfaceType::RightFootContact)
    {
        const Pose3f p = robotModel.soleRight;
        Pose3f c = p + Vector3f(30.f, -5.62f, 0.f);
        pose_ = WTO * sva::PTransform(c.rotation, c.translation);
    }
}

sva::PTransform Contact::anklePose(const RobotModel &model, const sva::PTransform &WTO)
{
    assert(halfLength_ != -1.f);
    assert(halfWidth_ != -1.f);

    sva::PTransform anklePose;
    if (surfaceType_ == SurfaceType::LeftFootContact)
    {
        const Pose3f ankle = model.limbs[Limbs::footLeft];
        anklePose = WTO * sva::PTransform(ankle.rotation, ankle.translation);
    }
    else if (surfaceType_ == SurfaceType::RightFootContact)
    {
        const Pose3f ankle = model.limbs[Limbs::footRight];
        anklePose = WTO * sva::PTransform(ankle.rotation, ankle.translation);
    }
    return anklePose;
}
