#include "Soccer.h"
#include "Tools/Module/ModuleManager.h"

Soccer::Soccer() : Cabsl<Soccer>(&activationGraph)
{
}

void Soccer::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
}

void Soccer::execute()
{
    update();
    beginFrame(theFrameInfo->time);
    Cabsl<Soccer>::execute(OptionInfos::getOption("Root"));
    endFrame();
}
