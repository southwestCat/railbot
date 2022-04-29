#pragma once

#include "Tools/Module/Blackboard.h"
#include "Tools/Module/BlackboardThread.h"
// #include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
// #include "Tools/Module/ModuleManager.h"

#include <iostream>

// class Adapter : public Provider
// {
// protected:
//     Adapter(Blackboard *bb) : Provider(bb) {}
// public:
//     virtual void tick();
// };

class Adapter
{
protected:
    BlackboardThread *blackboard;
    Blackboard bb;

    /** Provider */
    // NaoProvider naoProvider;
    // ModuleManager moduleManager;

    Adapter(BlackboardThread *bb) : blackboard(bb)
    {
        // Blackboard::setInstance(&this->bb);
        // NaoProvider::setInstance(&naoProvider);
        // ModuleManager::setInstance(&moduleManager);
    }

public:
    virtual void tick() {}
    virtual void beforeFrame() {}
    virtual void beforeModules() {}
    virtual void updateModules() {}
    virtual void afterModules() {}
    virtual void afterFrame() {}
};