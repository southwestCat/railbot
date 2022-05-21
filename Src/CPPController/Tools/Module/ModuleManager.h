#pragma once

#include <string>
#include "Macros.h"

#define UPDATE_REPRESENTATION(representation) ModuleManager::getInstance().updateRepresentation(CLASS2STRING(representation));

class ModuleManager
{
public:
    ModuleManager();
    ~ModuleManager();

    static thread_local ModuleManager *theInstance;
    static void setInstance(ModuleManager *instance);
    static ModuleManager &getInstance();

    static void updateRepresentation(std::string representation);

private:
    void *theInertialDataProvider = nullptr;
    void *theMotionCombinator = nullptr;
    void *theHeadMotionEngine = nullptr;
    void *theHeadMotionCombinator = nullptr;
    void *theJointAnglesProvider = nullptr;
    void *theMotionSelector = nullptr;
    void *theLegMotionCombinator = nullptr;
    void *theStandEngine = nullptr;
    void *theSpecialActionEngine = nullptr;
    void *theSitDownEngine = nullptr;
    void *theBalanceEngine = nullptr;
    void *theLIPMController = nullptr;
    void *theRobotModelProvider = nullptr;
    void *theLEDHandler = nullptr;
    void *theConfigurationsProvider = nullptr;
    void *theFsrFilteredDataProvider = nullptr;
};