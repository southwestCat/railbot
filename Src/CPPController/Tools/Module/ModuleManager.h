#pragma once

#include <string>
#include "Macros.h"

#define UPDATE_REPRESENTATION(representation) ModuleManager::getInstance().updateRepresentation(CLASS2STRING(representation));

class ModuleManager
{
public:
    ModuleManager();

    static thread_local ModuleManager *theInstance;
    static void setInstance(ModuleManager *instance);
    static ModuleManager &getInstance();

    static void updateRepresentation(std::string representation);
};