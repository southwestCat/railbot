#include "ModuleManager.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
#include "Modules/MotionControl/MotionCombinator/MotionCombinator.h"
#include "Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.h"
#include "Modules/MotionControl/MotionCombinator/HeadMotionCombinator.h"
#include "Modules/Infrastructure/JointAnglesProvider/JointAnglesProvider.h"

#define UPDATE_REPRESENTATION_WITH_PROVIDER(representation, provider)                                           \
    if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(representation)])                         \
    {                                                                                                           \
        representation *_the##representation = (representation *)Blackboard::getInstance().the##representation; \
        assert(_the##representation != nullptr);                                                                \
        provider *_the##provider = (provider *)ModuleManager::getInstance().the##provider;                      \
        assert(_the##provider != nullptr);                                                                      \
        _the##provider->update(*_the##representation);                                                          \
        Blackboard::getInstance().updatedRepresentation[CLASS2STRING(representation)] = true;                   \
    }

thread_local ModuleManager *ModuleManager::theInstance = nullptr;

ModuleManager::ModuleManager()
{
    theInstance = this;

    theInertialDataProvider = new InertialDataProvider;
    theMotionCombinator = new MotionCombinator;
    theHeadMotionEngine = new HeadMotionEngine;
    theHeadMotionCombinator = new HeadMotionCombinator;
    theJointAnglesProvider = new JointAnglesProvider;
}

ModuleManager::~ModuleManager()
{
    theInstance = nullptr;

    if (theInertialDataProvider != nullptr)
        delete (InertialDataProvider *)theInertialDataProvider;
    if (theMotionCombinator != nullptr)
        delete (MotionCombinator *)theMotionCombinator;
    if (theHeadMotionEngine != nullptr)
        delete (HeadMotionEngine *)theHeadMotionEngine;
    if (theHeadMotionCombinator != nullptr)
        delete (HeadMotionCombinator *)theHeadMotionCombinator;
    if (theJointAnglesProvider != nullptr)
        delete (JointAnglesProvider *)theJointAnglesProvider;
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
    else if (representation == CLASS2STRING(JointSensorData))
    {
        if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(JointSensorData)])
        {
            JointSensorData *_theJointSensorData = (JointSensorData *)Blackboard::getInstance().theJointSensorData;
            assert(_theJointSensorData != nullptr);
            NaoProvider::getInstance().update(*_theJointSensorData);
            Blackboard::getInstance().updatedRepresentation[CLASS2STRING(JointSensorData)] = true;
        }
    }
    else if (representation == CLASS2STRING(KeyStates))
    {
        if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(KeyStates)])
        {
            KeyStates *_theKeyStates = (KeyStates *)Blackboard::getInstance().theKeyStates;
            assert(_theKeyStates != nullptr);
            NaoProvider::getInstance().update(*_theKeyStates);
            Blackboard::getInstance().updatedRepresentation[CLASS2STRING(KeyStates)] = true;
        }
    }
    else if (representation == CLASS2STRING(JointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(JointRequest, MotionCombinator);
    }
    else if (representation == CLASS2STRING(HeadMotionEngineOutput))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(HeadMotionEngineOutput, HeadMotionEngine);
    }
    else if (representation == CLASS2STRING(HeadJointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(HeadJointRequest, HeadMotionCombinator);
    }
    else if (representation == CLASS2STRING(JointAngles))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(JointAngles, JointAnglesProvider);
    }
    else if (representation == CLASS2STRING(StiffnessSettings))
    {
        // Configurations
    }
    else if (representation == CLASS2STRING(HeadLimits))
    {
        // Configurations
    }
    else if (representation == CLASS2STRING(HeadMotionRequest))
    {
        // Modified in cognition
    }
    else
    {
        std::cout << "[ERROR]: Cannot find provider for representation: " << representation << std::endl;
    }
}
