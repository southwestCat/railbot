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
