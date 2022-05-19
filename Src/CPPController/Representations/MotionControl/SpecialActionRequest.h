#pragma once

class SpecialActionRequest
{
public:
    enum SpecialActionID
    {
        playDead,
        sitDown,
        stand,
        standHigh,
        standHighLookUp,
        getUpEngineDummy, // Used for debugging motion of getEngine

        numOfSpecialActionIDs
    };

    SpecialActionID specialAction = playDead;
    bool mirror = false;
};