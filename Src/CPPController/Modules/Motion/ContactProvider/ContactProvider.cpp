#include "ContactProvider.h"
#include "Tools/Module/ModuleManager.h"

void ContactProvider::update()
{
    UPDATE_REPRESENTATION(FloatingBaseEstimation);
}

void ContactProvider::update(Contact &c)
{
    update();
}