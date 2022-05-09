#pragma once

#include "Representations/MotionControl/SpecialActionEngineOutput.h"

class SpecialActionEngineBase
{
public:
};

class SpecialActionEngine : public SpecialActionEngineBase
{
public:
    void update(SpecialActionEngineOutput &s);
};
