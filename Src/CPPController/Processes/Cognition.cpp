#include "Cognition.h"

void Cognition::tick()
{
    resetUpdate();
    update();
    receive();
    send();
}

void Cognition::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedRepresentation;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}

void Cognition::update()
{
}

void Cognition::receive()
{
}

void Cognition::send()
{
}
