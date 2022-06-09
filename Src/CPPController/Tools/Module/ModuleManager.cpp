#include "ModuleManager.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
#include "Modules/MotionControl/MotionCombinator/MotionCombinator.h"
#include "Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.h"
#include "Modules/MotionControl/MotionCombinator/HeadMotionCombinator.h"
#include "Modules/Infrastructure/JointAnglesProvider/JointAnglesProvider.h"
#include "Modules/MotionControl/MotionSelector/MotionSelector.h"
#include "Modules/MotionControl/MotionCombinator/LegMotionCombinator.h"
#include "Modules/MotionControl/StandEngine/StandEngine.h"
#include "Modules/MotionControl/SpecialActionEngine/SpecialActionEngine.h"
#include "Modules/MotionControl/SitDownEngine/SitDownEngine.h"
#include "Modules/MotionControl/BalanceEngine/BalanceEngine.h"
#include "Modules/Motion/DCM/DCMController.h"
#include "Modules/Sensing/RobotModelProvider/RobotModelProvider.h"
#include "Modules/BehaviorControl/LEDHandler/LEDHandler.h"
#include "Modules/Configurations/ConfigurationsProvider.h"
#include "Modules/Sensing/FsrFilteredDataProvider/FsrFilteredDataProvider.h"
#include "Modules/Motion/MPC/FootstepsController.h"
#include "Modules/MotionControl/BalanceActionSelector/BalanceActionSelector.h"
#include "Modules/Motion/CoMProjectionObserver/CoMProjectionObserver.h"
#include "Modules/Motion/CoMCompliance/ComplianceController.h"

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

#define UPDATE_CONFIGURATION_WITH_PROVIDER(representation, provider)                                            \
    if (!Blackboard::getInstance().updatedConfig[CLASS2STRING(representation)])                                 \
    {                                                                                                           \
        representation *_the##representation = (representation *)Blackboard::getInstance().the##representation; \
        assert(_the##representation != nullptr);                                                                \
        provider *_the##provider = (provider *)ModuleManager::getInstance().the##provider;                      \
        assert(_the##provider != nullptr);                                                                      \
        _the##provider->update(*_the##representation);                                                          \
        Blackboard::getInstance().updatedConfig[CLASS2STRING(representation)] = true;                           \
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
    theMotionSelector = new MotionSelector;
    theLegMotionCombinator = new LegMotionCombinator;
    theStandEngine = new StandEngine;
    theSpecialActionEngine = new SpecialActionEngine;
    theSitDownEngine = new SitDownEngine;
    theBalanceEngine = new BalanceEngine;
    theDCMController = new DCMController;
    theRobotModelProvider = new RobotModelProvider;
    theLEDHandler = new LEDHandler;
    theConfigurationsProvider = new ConfigurationsProvider;
    theFsrFilteredDataProvider = new FsrFilteredDataProvider;
    theFootstepsController = new FootstepsController;
    theBalanceActionSelector = new BalanceActionSelector;
    theCoMProjectionObserver = new CoMProjectionObserver;
    theComplianceController = new ComplianceController;
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
    if (theMotionSelector != nullptr)
        delete (MotionSelector *)theMotionSelector;
    if (theLegMotionCombinator != nullptr)
        delete (LegMotionCombinator *)theLegMotionCombinator;
    if (theSpecialActionEngine != nullptr)
        delete (SpecialActionEngine *)theSpecialActionEngine;
    if (theSitDownEngine != nullptr)
        delete (SitDownEngine *)theSitDownEngine;
    if (theBalanceEngine != nullptr)
        delete (BalanceEngine *)theBalanceEngine;
    if (theDCMController != nullptr)
        delete (DCMController *)theDCMController;
    if (theRobotModelProvider != nullptr)
        delete (RobotModelProvider *)theRobotModelProvider;
    if (theLEDHandler != nullptr)
        delete (LEDHandler *)theLEDHandler;
    if (theConfigurationsProvider != nullptr)
        delete (ConfigurationsProvider *)theConfigurationsProvider;
    if (theFsrFilteredDataProvider != nullptr)
        delete (FsrFilteredDataProvider *)theFsrFilteredDataProvider;
    if (theFootstepsController != nullptr)
        delete (FootstepsController *)theFootstepsController;
    if (theBalanceActionSelector != nullptr)
        delete (BalanceActionSelector *)theBalanceActionSelector;
    if (theCoMProjectionObserver != nullptr)
        delete (CoMProjectionObserver *)theCoMProjectionObserver;
    if (theComplianceController != nullptr)
        delete (ComplianceController *)theComplianceController;
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
    else if (representation == CLASS2STRING(SystemSensorData))
    {
        if (!Blackboard::getInstance().updatedRepresentation[CLASS2STRING(SystemSensorData)])
        {
            SystemSensorData *_theSystemSensorData = (SystemSensorData *)Blackboard::getInstance().theSystemSensorData;
            assert(_theSystemSensorData != nullptr);
            NaoProvider::getInstance().update(*_theSystemSensorData);
            Blackboard::getInstance().updatedRepresentation[CLASS2STRING(SystemSensorData)] = true;
        }
    }
    else if (representation == CLASS2STRING(InertialData))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(InertialData, InertialDataProvider);
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
    else if (representation == CLASS2STRING(MotionInfo))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(MotionInfo, MotionCombinator);
    }
    else if (representation == CLASS2STRING(LegMotionSelection))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(LegMotionSelection, MotionSelector);
    }
    else if (representation == CLASS2STRING(LegJointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(LegJointRequest, LegMotionCombinator);
    }
    else if (representation == CLASS2STRING(StandEngineOuptut))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(StandEngineOuptut, StandEngine);
    }
    else if (representation == CLASS2STRING(SpecialActionEngineOutput))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(SpecialActionEngineOutput, SpecialActionEngine);
    }
    else if (representation == CLASS2STRING(SitDownEngineOutput))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(SitDownEngineOutput, SitDownEngine);
    }
    else if (representation == CLASS2STRING(BalanceEngineOutput))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(BalanceEngineOutput, BalanceEngine);
    }
    else if (representation == CLASS2STRING(DCMJointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(DCMJointRequest, DCMController);
    }
    else if (representation == CLASS2STRING(RobotModel))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(RobotModel, RobotModelProvider);
    }
    else if (representation == CLASS2STRING(LEDRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(LEDRequest, LEDHandler);
    }
    else if (representation == CLASS2STRING(FsrFilteredData))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(FsrFilteredData, FsrFilteredDataProvider);
    }
    else if (representation == CLASS2STRING(FootstepJointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(FootstepJointRequest, FootstepsController);
    }
    else if (representation == CLASS2STRING(BalanceActionSelection))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(BalanceActionSelection, BalanceActionSelector);
    }
    else if (representation == CLASS2STRING(CoMProjectionEstimation))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(CoMProjectionEstimation, CoMProjectionObserver);
    }
    else if (representation == CLASS2STRING(ComplianceJointRequest))
    {
        UPDATE_REPRESENTATION_WITH_PROVIDER(ComplianceJointRequest, ComplianceController);
    }
    else if (representation == CLASS2STRING(IMUCalibration))
    {
        UPDATE_CONFIGURATION_WITH_PROVIDER(IMUCalibration, ConfigurationsProvider);
    }
    else
    {
        std::cout << "[ERROR]: Cannot find provider for representation: " << representation << std::endl;
    }
}
