#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include "CABSL/Cabsl.h"
#include "Modules/BehaviorControl/BehaviorControl/Soccer.h"

class Cognition : public Adapter
{
public:
    Cognition(BlackboardThread *bb) : Adapter(bb)
    {
    }

    void tick();
    void send();
    void receive();
    void resetUpdate();

    void beforeFrame() override;
    void beforeModules() override;
    void updateModules() override;
    void afterModules() override;
    void afterFrame() override;

private:
    Soccer soccer;
};