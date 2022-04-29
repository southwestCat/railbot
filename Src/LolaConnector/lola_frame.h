#pragma once

#include <map>
#include <string>
#include <vector>

#include <msgpack.hpp>

#include "battery.h"
#include "fsr.h"
#include "imu.h"
#include "joints.h"
#include "leds.h"
#include "touch.h"
#include "sonar.h"
#include "robotConfig.h"

struct LolaSensorFrame {
    Joints joints;
    IMU imu;
    Sonar sonar;
    FSR fsr;
    Battery battery;
    Touch touch;
    RobotConfig robotConfig;
};

struct LolaActuatorFrame {
    Joints joints;
    Leds leds;
    // Robocup robocup;
};

class LolaFrameHandler {
public:
    LolaFrameHandler();
    const LolaSensorFrame& unpack(const char* const buffer, size_t size);
    std::pair<char*, size_t> pack();

    LolaActuatorFrame actuator_frame;

private:
    void initSensorFrame();
    void initActuatorFrame();

    LolaSensorFrame sensor_frame;
    std::map<std::string, std::vector<float*>> sensor_frame_positions;
    std::map<std::string, std::vector<float*>> actuator_frame_positions;
    std::map<std::string, std::vector<std::string*>> robotConfig_frame_positions;
    std::map<std::string, std::vector<int*>> status_frame_positions;
    // active in default, on/off limit implemented to keep joints in the acceptable range of acceleration.
    // std::map<std::string, std::vector<bool*>> filter_frame_positions;              
    msgpack::sbuffer buffer;
};
