#include "JointRequest.h"

JointRequest::JointRequest()
{
    angles.fill(off);
}

void JointRequest::mirror(const JointRequest &other)
{
    JointAngles::mirror(other);
    stiffnessData.mirror(other.stiffnessData);
}

Angle JointRequest::mirror(Joints::Joint joint) const
{
    return JointAngles::mirror(joint);
}

bool JointRequest::isValid(bool allowUseDefault) const
{
    bool isValid = true;
    for (unsigned i = 0; i < Joints::numOfJoints; i++)
        if (!std::isfinite(angles[i]))
        { 
            std::cout << "Joint " << i << " is invalid.\n";
            isValid = false;
        }
    return stiffnessData.isValid(allowUseDefault) && isValid;
}
