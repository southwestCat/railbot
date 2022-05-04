#include "ModuleManager.h"
#include "Modules/Infrastructure/NaoProvider.h"

#define UPDATE_REPRESENTATION_WITH_PROVIDER(representation, provider)                                           \
    if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(representation)])                         \
    {                                                                                                           \
        representation *_the##representation = (representation *)Blackboard::getInstance().the##representation; \
        assert(_the##representation != nullptr);                                                                \
        assert(ModuleManager::getInstance().the##provider != nullptr);                                          \
        ModuleManager::getInstance().the##provider.update(*_the##representation);                               \
        Blackboard::getInstance().updatedRepresentation[CLASS2STRING(representation)] = true;                   \
    }

thread_local ModuleManager *ModuleManager::theInstance = nullptr;

ModuleManager::ModuleManager()
{
    theInstance = this;
}

void ModuleManager::setInstance(ModuleManager *instance)
{
    theInstance = instance;
}

ModuleManager &ModuleManager::getInstance()
{
    return *theInstance;
}

void ModuleManager::updateRepresentation(std::string representation)
{
    if (representation == CLASS2STRING(FrameInfo))
    {
        FrameInfo *_theFrameInfo = (FrameInfo *)Blackboard::getInstance().theFrameInfo;
        assert(_theFrameInfo != nullptr);
        NaoProvider::getInstance().update(*_theFrameInfo);
    }
    else if (representation == CLASS2STRING(InertialSensorData))
    {
        if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(InertialSensorData)])
        {
            InertialSensorData *_theInertialSensorData = (InertialSensorData *)Blackboard::getInstance().theInertialSensorData;
            assert(_theInertialSensorData != nullptr);
            NaoProvider::getInstance().update(*_theInertialSensorData);
            Blackboard::getInstance().updatedRepresentation[CLASS2STRING(InertialSensorData)] = true;
        }
    }
    else if (representation == CLASS2STRING(FsrSensorData))
    {
        if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(FsrSensorData)])
        {
            FsrSensorData *_theFsrSensorData = (FsrSensorData *)Blackboard::getInstance().theFsrSensorData;
            assert(_theFsrSensorData != nullptr);
            NaoProvider::getInstance().update(*_theFsrSensorData);
            Blackboard::getInstance().updatedRepresentation[CLASS2STRING(FsrSensorData)] = true;
        }
    }
    else
    {
        std::cout << "[ERROR]: Cannot find provider for representation: " << representation << std::endl;
    }
}
