#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include "CABSL/Cabsl.h"

class Behavior : public Adapter
{
public:
    Behavior(BlackboardThread *bb) : Adapter(bb)
    {
    }

    void tick();
    void update();
    void send();
    void receive();
    void resetUpdate();
};