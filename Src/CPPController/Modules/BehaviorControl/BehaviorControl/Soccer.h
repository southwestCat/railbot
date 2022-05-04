#pragma once

#include "CABSL/Cabsl.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/Module/Blackboard.h"

class SoccerBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(KeyStates);
};

class Soccer : public SoccerBase, public Cabsl<Soccer>
{
public:
    Soccer();
    void update();
    void execute();
#include "Options.h"
    ActivationGraph activationGraph;
};
