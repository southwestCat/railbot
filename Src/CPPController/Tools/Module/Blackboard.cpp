#include "Blackboard.h"
#include "Macros.h"

#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/FootTask.h"
#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/NetWrenchEstimation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Infrastructure/Stiffness.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/SpecialActionEngineOutput.h"
#include "Representations/Configuration/JointLimits.h"

static thread_local Blackboard *theInstance = nullptr;

Blackboard::Blackboard()
{
    theInstance = this;

    theFrameInfo = new FrameInfo;
    insert(CLASS2STRING(FrameInfo));

    theJointRequest = new JointRequest;
    insert(CLASS2STRING(JointRequest));

    theInertialSensorData = new InertialSensorData;
    insert(CLASS2STRING(InertialSensorData));

    theInertialData = new InertialData;
    insert(CLASS2STRING(InertialData));

    theJointSensorData = new JointSensorData;
    insert(CLASS2STRING(JointSensorData));

    theKeyStates = new KeyStates;
    insert(CLASS2STRING(KeyStates));

    theHeadMotionRequest = new HeadMotionRequest;
    insert(CLASS2STRING(HeadMotionRequest));

    theJointAngles = new JointAngles;
    insert(CLASS2STRING(JointAngles));

    theFsrSensorData = new FsrSensorData;
    insert(CLASS2STRING(FsrSensorData));

    theHeadMotionEngineOutput = new HeadMotionEngineOutput;
    insert(CLASS2STRING(HeadMotionEngineOutput));

    theHeadJointRequest = new HeadJointRequest;
    insert(CLASS2STRING(HeadJointRequest));

    theMotionInfo = new MotionInfo;
    insert(CLASS2STRING(MotionInfo));

    theMotionRequest = new MotionRequest;
    insert(CLASS2STRING(MotionRequest));

    theLegMotionSelection = new LegMotionSelection;
    insert(CLASS2STRING(LegMotionSelection));

    theWalkingEngineOutput = new WalkingEngineOutput;
    insert(CLASS2STRING(WalkingEngineOutput));

    theStandEngineOuptut = new StandEngineOuptut;
    insert(CLASS2STRING(StandEngineOuptut));

    theRobotModel = new RobotModel;
    insert(CLASS2STRING(RobotModel));

    theLegJointRequest = new LegJointRequest;
    insert(CLASS2STRING(LegJointRequest));

    theSpecialActionEngineOutput = new SpecialActionEngineOutput;
    insert(CLASS2STRING(SpecialActionEngineOutput));

    theStiffnessSettings = new StiffnessSettings;
    insert(CLASS2STRING(StiffnessSettings), configMap);

    theJointLimits = new JointLimits;
    insert(CLASS2STRING(JointLimits), configMap);

    theRobotDimensions = new RobotDimensions;
    insert(CLASS2STRING(RobotDimensions), configMap);

    theHeadLimits = new HeadLimits;
    insert(CLASS2STRING(HeadLimits), configMap);
}

Blackboard::~Blackboard()
{
    theInstance = nullptr;

    if (theFrameInfo != nullptr)
        delete (FrameInfo *)theFrameInfo;
    if (theJointRequest != nullptr)
        delete (JointRequest *)theJointRequest;
    if (theInertialSensorData != nullptr)
        delete (InertialSensorData *)theInertialSensorData;
    if (theKeyStates != nullptr)
        delete (KeyStates *)theKeyStates;
    if (theHeadMotionRequest != nullptr)
        delete (HeadMotionRequest *)theHeadMotionRequest;
    if (theHeadLimits != nullptr)
        delete (HeadLimits *)theHeadLimits;
    if (theJointAngles != nullptr)
        delete (JointAngles *)theJointAngles;
    if (theFsrSensorData != nullptr)
        delete (FsrSensorData *)theFsrSensorData;
    if (theHeadMotionEngineOutput != nullptr)
        delete (HeadMotionEngineOutput *)theHeadMotionEngineOutput;
    if (theHeadJointRequest != nullptr)
        delete (HeadJointRequest *)theHeadJointRequest;
    if (theStiffnessSettings != nullptr)
        delete (StiffnessSettings *)theStiffnessSettings;
    if (theMotionInfo != nullptr)
        delete (MotionInfo *)theMotionInfo;
    if (theMotionRequest != nullptr)
        delete (MotionRequest *)theMotionRequest;
    if (theLegMotionSelection != nullptr)
        delete (LegMotionSelection *)theLegMotionSelection;
    if (theWalkingEngineOutput != nullptr)
        delete (WalkingEngineOutput *)theWalkingEngineOutput;
    if (theStandEngineOuptut != nullptr)
        delete (StandEngineOuptut *)theStandEngineOuptut;
    if (theRobotModel != nullptr)
        delete (RobotModel *)theRobotModel;
    if (theRobotDimensions != nullptr)
        delete (RobotDimensions *)theRobotDimensions;
    if (theLegJointRequest != nullptr)
        delete (LegJointRequest *)theLegJointRequest;
    if (theSpecialActionEngineOutput != nullptr)
        delete (SpecialActionEngineOutput *)theSpecialActionEngineOutput;
    if (theJointLimits != nullptr)
        delete (JointLimits *)theJointLimits;
}

Blackboard &Blackboard::getInstance()
{
    return *theInstance;
}

void Blackboard::setInstance(Blackboard *instance)
{
    theInstance = instance;
}

// void Blackboard::initMap()
// {
//     initRepresentationMap();
//     initConfigMap();
// }

// void Blackboard::initRepresentationMap()
// {
// }

// void Blackboard::initConfigMap()
// {
// }

void Blackboard::insert(std::string string, MapType type)
{
    // Default std::map::insert will ignore same element.
    // this exist() is not necessary, but maybe some help, e.g.
    // want to inert representationMap, but inert configMap
    if (!exists(string))
    {
        if (type == configMap)
        {
            updatedConfig.insert(std::pair<std::string, bool>(string, false));
        }
        else if (type == representationMap)
        {
            updatedRepresentation.insert(std::pair<std::string, bool>(string, false));
        }
        else
        {
            std::cerr << "No this type map" << std::endl;
        }
    }
}

bool Blackboard::exists(std::string representation)
{
    return (updatedRepresentation.find(representation) != updatedRepresentation.end()) || (updatedConfig.find(representation) != updatedConfig.end());
}

void Blackboard::setRMap(std::string representation)
{
    if (!exists(representation))
        updatedRepresentation.insert(std::pair<std::string, bool>(representation, false));
    updatedRepresentation[representation] = true;
}

void Blackboard::setCMap(std::string config)
{
    if (!exists(config))
        updatedConfig.insert(std::pair<std::string, bool>(config, false));
    updatedConfig[config] = true;
}
