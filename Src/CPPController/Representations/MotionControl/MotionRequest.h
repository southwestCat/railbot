#pragma once

class MotionRequest
{
public:
    enum Motion
    {
        walk,
        stand,
        specialActions,
        numOfMotions
    };

    Motion motion = specialActions;
};
