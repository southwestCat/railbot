#pragma once

class MotionRequest
{
public:
    enum Motion
    {
        walk,
        stand,
        balance,
        specialAction,
        numOfMotions
    };

    Motion motion = specialAction;
};
