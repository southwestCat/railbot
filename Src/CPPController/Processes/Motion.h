#pragma once

#include "Modules/Infrastructure/Adapter.h"

class Motion : public Adapter 
{
public:
    Motion(BlackboardThread* bb) : Adapter(bb) 
    {
    }
    void tick();

    /* reset blackboard updatedRepresentation to false to make sure all representations are not updated 
     * before update();
     */
    void resetUpdate();
    void send();
    void receive();

    void beforeFrame() override;
    void beforeModules() override;
    void updateModules() override;
    void afterModules() override;
    void afterFrame() override;
};