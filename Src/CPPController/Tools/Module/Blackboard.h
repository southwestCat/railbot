#pragma once

#include <vector>
#include <map>
#include <cassert>

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

    //! LOG
    std::vector<float> logLeftPitchd;

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