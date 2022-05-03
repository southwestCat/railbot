#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include "CABSL/Cabsl.h"

class Cognition : public Adapter
{
public:
    Cognition(BlackboardThread *bb) : Adapter(bb)
    {
    }

    void tick();
    void update();
    void send();
    void receive();
    void resetUpdate();
};