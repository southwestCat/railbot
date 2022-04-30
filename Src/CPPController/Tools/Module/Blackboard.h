#pragma once

#include <vector>

#define REQUIRES_REPRESENTATION(representation)                                                                  \
    const representation *the##representation = (representation *)Blackboard::getInstance().the##representation; \
    const bool update##representation = true;

#define USES_REPRESENTATION(representation)                                                                      \
    const representation *the##representation = (representation *)Blackboard::getInstance().the##representation; \
    const bool update##representation = false;

class Blackboard
{
public:
    Blackboard();
    ~Blackboard();

    static Blackboard &getInstance();
    static void setInstance(Blackboard *instance);
    const float dt() const { return timeStamp_; }

    float timeStamp_ = 0.01f;

    void *theFrameInfo = nullptr;
    // FsrSensorData theFsrSensorData;
    // InertialData theInertialData;
    // InertialSensorData theInertialSensorData;
    // JointLimits theJointLimits;
    void *theJointRequest = nullptr;
    // JointSensorData theJointSensorData;
    // MassCalibration theMassCalibration;
    // NetWrenchEstimation theNetWrenchEstimation;
    // FloatingBaseEstimation theFloatingBaseEstimation;
    // RobotDimensions theRobotDimensions;
    // RobotModel theRobotModel;
    // FootTask theFootTask[2]; //< 0: left, 1: right

    //! LOG
    std::vector<float> logLeftPitchd;
};