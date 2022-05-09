#include "SpecialActionEngine.h"

void SpecialActionEngine::update(SpecialActionEngineOutput &s)
{
    //! Play dead.
    s.angles.fill(JointAngles::off);
    s.isLeavingPossible = true;
}