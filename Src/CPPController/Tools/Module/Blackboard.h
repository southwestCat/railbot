#pragma once

#include <vector>
#include <map>
#include <cassert>

#define REQUIRES_REPRESENTATION(representation)                                                                  \
    const representation *the##representation = (representation *)Blackboard::getInstance().the##representation; \
    const bool update##representation = true;

#define USES_REPRESENTATION(representation)                                                                \
    const representation *the##representation = (representation *)Blackboard::getInstance().the##representation; \
    const bool update##representation = false;

#define MODIFIES_REPRESENTATION(representation)                                                            \
    representation *the##representation = (representation *)Blackboard::getInstance().the##representation; \
    const bool update##representation = false;

class Blackboard
{
public:
    Blackboard();
    ~Blackboard();

    static Blackboard &getInstance();
    static void setInstance(Blackboard *instance);
    float dt() const { return timeStamp_; }

    float timeStamp_ = 0.01f;

    void *theFrameInfo = nullptr;
    void *theKeyStates = nullptr;
    void *theFsrSensorData = nullptr;
    void *theInertialData = nullptr;
    void *theInertialSensorData = nullptr;
    // JointLimits theJointLimits;
    void *theJointRequest = nullptr;
    void *theJointSensorData = nullptr;
    void *theHeadMotionRequest = nullptr;
    void *theHeadLimits = nullptr;
    void *theJointAngles = nullptr;
    void *theHeadMotionEngineOutput = nullptr;
    void *theHeadJointRequest = nullptr;
    void *theStiffnessSettings = nullptr;
    void *theMotionInfo = nullptr;
    void *theMotionRequest = nullptr;
    void *theLegMotionSelection = nullptr;
    void *theWalkingEngineOutput = nullptr;
    void *theStandEngineOuptut = nullptr;
    void *theRobotModel = nullptr;
    void *theRobotDimensions = nullptr;
    void *theLegJointRequest = nullptr;
    void *theSpecialActionEngineOutput = nullptr;
    void *theJointLimits = nullptr;
    void *theSitDownEngineOutput = nullptr;
    void *theBalanceEngineOutput = nullptr;
    void *theFloatingBaseEstimation = nullptr;
    void *theStabilizerJointRequest = nullptr;
    void *theNetWrenchEstimation = nullptr;
    void *theMassCalibration = nullptr;
    void *theLeftFootTask = nullptr;
    void *theRightFootTask = nullptr;
    void *theLEDRequest = nullptr;
    void *theRobotInfo = nullptr;
    void *theIMUCalibration = nullptr;
    void *theSystemSensorData = nullptr;
    void *theFsrFilteredData = nullptr;
    void *theFootstepJointRequest = nullptr;
    void *theMPCControllerState = nullptr;
    void *theBalanceActionSelection = nullptr;
    void *theCoMProjectionEstimation = nullptr;
    void *theComplianceJointRequest = nullptr;

public:
    std::map<std::string, bool> updatedRepresentation;
    std::map<std::string, bool> updatedConfig;
    bool exists(std::string representation);

private:
    enum MapType
    {
        representationMap,
        configMap
    };

    // void initMap();
    // void initRepresentationMap();
    // void initConfigMap();
    void setRMap(std::string representation);
    void setCMap(std::string config);
    void insert(std::string string, MapType type = representationMap);
};