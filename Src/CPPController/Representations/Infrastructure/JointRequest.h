#pragma once

#include "JointAngles.h"
#include "Stiffness.h"

class JointRequest : public JointAngles
{
public:
    JointRequest();

    /** Initializes this instance with the mirrored data of other. */
    void mirror(const JointRequest &other);

    /** Returns the mirrored angle of joint. */
    Angle mirror(Joints::Joint joint) const;

    /** Checkes if the JointRequest is valide. */
    bool isValid(bool allowUseDefault = true) const;

    StiffnessData stiffnessData; /**< the stiffness for all joints */
};

class HeadJointRequest : public JointRequest
{
};

class LegJointRequest : public JointRequest
{
};

class ArmJointRequest : public JointRequest
{
};

class StabilizerJointRequest : public JointRequest
{
};

class DCMJointRequest : public JointRequest
{
};

class ComplianceJointRequest : public JointRequest
{
};

class FootstepJointRequest : public JointRequest
{
};
