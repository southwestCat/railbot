#include "Motion.h"
#include <iostream>

void Motion::tick()
{
    beforeFrame();
    beforeModules();
    updateModules();
    afterModules();
    afterFrame();
}

void Motion::beforeFrame()
{

}

void Motion::beforeModules()
{

}

void Motion::updateModules()
{
    
}

void Motion::afterModules()
{

}

void Motion::afterFrame()
{

}



