#include "Blackboard.h"

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

static Blackboard *theInstance = nullptr;

Blackboard::Blackboard()
{
    theInstance = this;

    theFrameInfo = new FrameInfo;
    theJointRequest = new JointRequest;
}

Blackboard::~Blackboard()
{
    theInstance = nullptr;

    if (theFrameInfo != nullptr)
        delete (FrameInfo *)theFrameInfo;
    if (theJointRequest != nullptr)
        delete (JointRequest *)theJointRequest;
}

Blackboard &Blackboard::getInstance()
{
    return *theInstance;
}

void Blackboard::setInstance(Blackboard *instance)
{
    theInstance = instance;
}