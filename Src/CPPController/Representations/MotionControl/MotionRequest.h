#pragma once

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
};
