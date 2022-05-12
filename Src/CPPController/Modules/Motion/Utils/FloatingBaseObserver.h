#pragma once

#include "Representations/Sensing/FloatingBaseEstimation.h"

class FloatingBaseObserverBase
{
public:
};

class FloatingBaseObserver : public FloatingBaseObserverBase
{
public:
    void update(FloatingBaseEstimation &f);
private:
    void update();
};
