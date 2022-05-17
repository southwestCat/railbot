#pragma once

#define PENALTY_NONE 0
#define PENALTY_SPL_REQUEST_FOR_PICKUP 8

#define STATE_INITIAL 0
#define STATE_READY 1
#define STATE_SET 2
#define STATE_PLAYING 3
#define STATE_FINISHED 4

class RobotInfo
{
public:
    unsigned int penalty;
    unsigned int state;
};