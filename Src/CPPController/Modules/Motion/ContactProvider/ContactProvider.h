#pragma once

#include "Modules/Motion/Utils/Contact.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Tools/Module/Blackboard.h"

class ConatctProviderBase
{
public:
    REQUIRES_REPRESENTATION(FloatingBaseEstimation);
};

class ContactProvider : public ConatctProviderBase
{
public:
    void update(Contact &c);
private:
    void update();
};
