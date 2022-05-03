#include "Behavior.h"

void Behavior::tick()
{
    resetUpdate();
    update();
    receive();
    send();
}

void Behavior::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedRepresentation;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}

void Behavior::update()
{
}

void Behavior::receive()
{
}

void Behavior::send()
{
}
