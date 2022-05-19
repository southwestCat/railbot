#pragma once

#include "SpecialActionRequest.h"

class MotionRequest
{
public:
    enum Motion
    {
        walk,
        stand,
        sitDown,
        balance,
        specialAction,
        numOfMotions
    };

    Motion motion = specialAction;
    SpecialActionRequest specialActionRequest;
};
