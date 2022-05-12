#include "NetWrenchObserver.h"
#include "Tools/Module/ModuleManager.h"

void NetWrenchObserver::update()
{
    UPDATE_REPRESENTATION(Contact);
}

void NetWrenchObserver::update(NetWrenchEstimation &n)
{
    update();
}
