#include "BalanceEngine.h"
#include "Tools/Module/ModuleManager.h"

void BalanceEngine::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(JointAngles);
}

void BalanceEngine::update(BalanceEngineOutput &o)
{
    update();

    printf("in BalanceEngine.\n");
}