#include "lola_frame.h"

#include <iostream>

#include "stl_ext.h"

using namespace std;

LolaFrameHandler::LolaFrameHandler() {
    initSensorFrame();
    initActuatorFrame();
}

void LolaFrameHandler::initSensorFrame() {
    auto& gyr = sensor_frame.imu.gyr;
    auto& accel = sensor_frame.imu.accel;
    auto& angles = sensor_frame.imu.angles;
    auto& head = sensor_frame.joints.head;
    auto& arms = sensor_frame.joints.arms;
    auto& legs = sensor_frame.joints.legs;
    auto& fsr = sensor_frame.fsr;
    auto& battery = sensor_frame.battery;
    auto& touch = sensor_frame.touch;
    auto& sonar = sensor_frame.sonar;
    auto& robotConfig = sensor_frame.robotConfig;
    // Those aren't all sensors yet, you can just add them according to Softbank's documentation.
    sensor_frame_positions = {
        {"Accelerometer", {&accel.x, &accel.y, &accel.z}},
        {"Gyroscope", {&gyr.roll, &gyr.pitch, &gyr.yaw}},
        {"Angles",{&angles.X, &angles.Y, &angles.Z}},
        {"Sonar",{&sonar.left, &sonar.right}},
        // The order for battery values in Softbank's documentation is wrong.
        {"Battery", {&battery.charge, &battery.status, &battery.current, &battery.temp}},
        {"FSR", {&fsr.left.fl, &fsr.left.fr, &fsr.left.rl, &fsr.left.rr,
                 &fsr.right.fl, &fsr.right.fr, &fsr.right.rl, &fsr.right.rr}},
        {"Position", {
             &head[HeadYaw].angle, &head[HeadPitch].angle,
             &arms[LShoulderPitch].angle, &arms[LShoulderRoll].angle, &arms[LElbowYaw].angle,
             &arms[LElbowRoll].angle, &arms[LWristYaw].angle,
             &legs[HipYawPitch].angle, &legs[LHipRoll].angle, &legs[LHipPitch].angle,
             &legs[LKneePitch].angle, &legs[LAnklePitch].angle, &legs[LAnkleRoll].angle,
             &legs[RHipRoll].angle,  &legs[RHipPitch].angle, &legs[RKneePitch].angle,
             &legs[RAnklePitch].angle, &legs[RAnkleRoll].angle,
             &arms[RShoulderPitch].angle, &arms[RShoulderRoll].angle, &arms[RElbowYaw].angle,
             &arms[RElbowRoll].angle, &arms[RWristYaw].angle,
             &arms[LHand].angle, &arms[RHand].angle}},
        {"Stiffness", {
             &head[HeadYaw].stiffness, &head[HeadPitch].stiffness,
             &arms[LShoulderPitch].stiffness, &arms[LShoulderRoll].stiffness, &arms[LElbowYaw].stiffness,
             &arms[LElbowRoll].stiffness, &arms[LWristYaw].stiffness,
             &legs[HipYawPitch].stiffness, &legs[LHipRoll].stiffness, &legs[LHipPitch].stiffness,
             &legs[LKneePitch].stiffness, &legs[LAnklePitch].stiffness, &legs[LAnkleRoll].stiffness,
             &legs[RHipRoll].stiffness,  &legs[RHipPitch].stiffness, &legs[RKneePitch].stiffness,
             &legs[RAnklePitch].stiffness, &legs[RAnkleRoll].stiffness,
             &arms[RShoulderPitch].stiffness, &arms[RShoulderRoll].stiffness, &arms[RElbowYaw].stiffness,
             &arms[RElbowRoll].stiffness, &arms[RWristYaw].stiffness,
             &arms[LHand].stiffness, &arms[RHand].stiffness}},
        {"Temperature", {
             &head[HeadYaw].temperature, &head[HeadPitch].temperature,
             &arms[LShoulderPitch].temperature, &arms[LShoulderRoll].temperature, &arms[LElbowYaw].temperature,
             &arms[LElbowRoll].temperature, &arms[LWristYaw].temperature,
             &legs[HipYawPitch].temperature, &legs[LHipRoll].temperature, &legs[LHipPitch].temperature,
             &legs[LKneePitch].temperature, &legs[LAnklePitch].temperature, &legs[LAnkleRoll].temperature,
             &legs[RHipRoll].temperature,  &legs[RHipPitch].temperature, &legs[RKneePitch].temperature,
             &legs[RAnklePitch].temperature, &legs[RAnkleRoll].temperature,
             &arms[RShoulderPitch].temperature, &arms[RShoulderRoll].temperature, &arms[RElbowYaw].temperature,
             &arms[RElbowRoll].temperature, &arms[RWristYaw].temperature,
             &arms[LHand].temperature, &arms[RHand].temperature}},
        {"Current", {
             &head[HeadYaw].current, &head[HeadPitch].current,
             &arms[LShoulderPitch].current, &arms[LShoulderRoll].current, &arms[LElbowYaw].current,
             &arms[LElbowRoll].current, &arms[LWristYaw].current,
             &legs[HipYawPitch].current, &legs[LHipRoll].current, &legs[LHipPitch].current,
             &legs[LKneePitch].current, &legs[LAnklePitch].current, &legs[LAnkleRoll].current,
             &legs[RHipRoll].current,  &legs[RHipPitch].current, &legs[RKneePitch].current,
             &legs[RAnklePitch].current, &legs[RAnkleRoll].current,
             &arms[RShoulderPitch].current, &arms[RShoulderRoll].current, &arms[RElbowYaw].current,
             &arms[RElbowRoll].current, &arms[RWristYaw].current,
             &arms[LHand].current, &arms[RHand].current}},
        {"Touch", {
            &touch.ChestBtn, &touch.headTouch.front, &touch.headTouch.middle, &touch.headTouch.rear,
            &touch.lFootBumper.left, &touch.lFootBumper.right, &touch.lHandTouch.back, &touch.lHandTouch.left,
            &touch.lHandTouch.right, &touch.rFootBumper.left, &touch.rFootBumper.right, &touch.rHandTouch.back,
            &touch.rHandTouch.left, &touch.rHandTouch.right}}
    };
    robotConfig_frame_positions = {{"RobotConfig",{&robotConfig.BodyId, &robotConfig.BodyVersion, &robotConfig.HeadId, &robotConfig.HeadVersion}}};
    status_frame_positions = {{"Status", {&head[HeadYaw].status, &head[HeadPitch].status,
            &arms[LShoulderPitch].status, &arms[LShoulderRoll].status, &arms[LElbowYaw].status,
            &arms[LElbowRoll].status, &arms[LWristYaw].status,
            &legs[HipYawPitch].status, &legs[LHipRoll].status, &legs[LHipPitch].status,
            &legs[LKneePitch].status, &legs[LAnklePitch].status, &legs[LAnkleRoll].status,
            &legs[RHipRoll].status,  &legs[RHipPitch].status, &legs[RKneePitch].status,
            &legs[RAnklePitch].status, &legs[RAnkleRoll].status,
            &arms[RShoulderPitch].status, &arms[RShoulderRoll].status, &arms[RElbowYaw].status,
            &arms[RElbowRoll].status, &arms[RWristYaw].status,
            &arms[LHand].status, &arms[RHand].status}}};
}

void LolaFrameHandler::initActuatorFrame() {
    auto& head = actuator_frame.joints.head;
    auto& arms = actuator_frame.joints.arms;
    auto& legs = actuator_frame.joints.legs;
    auto& ears = actuator_frame.leds.ears;
    auto& eyes = actuator_frame.leds.eyes;
    auto& footsLed = actuator_frame.leds.foots;
    auto& chestLed = actuator_frame.leds.chestLed;
    auto& headLed = actuator_frame.leds.head;
    // auto& safety = actuator_frame.robocup.safety;
    // Also not all actuators, you know the drill. ;)
    actuator_frame_positions = {
        {"LEar", {&ears.left[0], &ears.left[1], &ears.left[2], &ears.left[3], &ears.left[4],
                  &ears.left[5], &ears.left[6], &ears.left[7], &ears.left[8], &ears.left[9]}},
        {"REar", {&ears.right[9], &ears.right[8], &ears.right[7], &ears.right[6], &ears.right[5],
                  &ears.right[4], &ears.right[3], &ears.right[2], &ears.right[1], &ears.right[0]}},
        // What did they take to come up with the ordering of the eye LEDs?
        {"LEye", {&eyes.left[1].r, &eyes.left[0].r, &eyes.left[7].r, &eyes.left[6].r,
                  &eyes.left[5].r, &eyes.left[4].r, &eyes.left[3].r, &eyes.left[2].r,
                  &eyes.left[1].g, &eyes.left[0].g, &eyes.left[7].g, &eyes.left[6].g,
                  &eyes.left[5].g, &eyes.left[4].g, &eyes.left[3].g, &eyes.left[2].g,
                  &eyes.left[1].b, &eyes.left[0].b, &eyes.left[7].b, &eyes.left[6].b,
                  &eyes.left[5].b, &eyes.left[4].b, &eyes.left[3].b, &eyes.left[2].b}},
        {"REye", {&eyes.right[0].r, &eyes.right[1].r, &eyes.right[2].r, &eyes.right[3].r,
                  &eyes.right[4].r, &eyes.right[5].r, &eyes.right[6].r, &eyes.right[7].r,
                  &eyes.right[0].g, &eyes.right[1].g, &eyes.right[2].g, &eyes.right[3].g,
                  &eyes.right[4].g, &eyes.right[5].g, &eyes.right[6].g, &eyes.right[7].g,
                  &eyes.right[0].b, &eyes.right[1].b, &eyes.right[2].b, &eyes.right[3].b,
                  &eyes.right[4].b, &eyes.right[5].b, &eyes.right[6].b, &eyes.right[7].b}},
        {"Position", {
             &head[HeadYaw].angle, &head[HeadPitch].angle,
             &arms[LShoulderPitch].angle, &arms[LShoulderRoll].angle, &arms[LElbowYaw].angle,
             &arms[LElbowRoll].angle, &arms[LWristYaw].angle,
             &legs[HipYawPitch].angle, &legs[LHipRoll].angle, &legs[LHipPitch].angle,
             &legs[LKneePitch].angle, &legs[LAnklePitch].angle, &legs[LAnkleRoll].angle,
             &legs[RHipRoll].angle,  &legs[RHipPitch].angle, &legs[RKneePitch].angle,
             &legs[RAnklePitch].angle, &legs[RAnkleRoll].angle,
             &arms[RShoulderPitch].angle, &arms[RShoulderRoll].angle, &arms[RElbowYaw].angle,
             &arms[RElbowRoll].angle, &arms[RWristYaw].angle,
             &arms[LHand].angle, &arms[RHand].angle}},
        {"Stiffness", {
             &head[HeadYaw].stiffness, &head[HeadPitch].stiffness,
             &arms[LShoulderPitch].stiffness, &arms[LShoulderRoll].stiffness, &arms[LElbowYaw].stiffness,
             &arms[LElbowRoll].stiffness, &arms[LWristYaw].stiffness,
             &legs[HipYawPitch].stiffness, &legs[LHipRoll].stiffness, &legs[LHipPitch].stiffness,
             &legs[LKneePitch].stiffness, &legs[LAnklePitch].stiffness, &legs[LAnkleRoll].stiffness,
             &legs[RHipRoll].stiffness,  &legs[RHipPitch].stiffness, &legs[RKneePitch].stiffness,
             &legs[RAnklePitch].stiffness, &legs[RAnkleRoll].stiffness,
             &arms[RShoulderPitch].stiffness, &arms[RShoulderRoll].stiffness, &arms[RElbowYaw].stiffness,
             &arms[RElbowRoll].stiffness, &arms[RWristYaw].stiffness,
             &arms[LHand].stiffness, &arms[RHand].stiffness}},
        {"Chest",{&chestLed.r, &chestLed.g, &chestLed.b}},
        {"LFoot",{&footsLed.left.r, &footsLed.left.g, &footsLed.left.b}},
        {"RFoot",{&footsLed.right.r, &footsLed.right.g, &footsLed.right.b}},
        {"Skull",{&headLed[10], &headLed[9],&headLed[11],&headLed[0],&headLed[1],&headLed[2]
                 ,&headLed[5],&headLed[4],&headLed[3],&headLed[6],&headLed[7],&headLed[8]}}
    };
    // filter_frame_positions = {{"Robocup", {&safety}}};
}

const LolaSensorFrame& LolaFrameHandler::unpack(const char* const buffer, size_t size) {
    msgpack::unpacker pac;
    pac.reserve_buffer(size);
    memcpy(pac.buffer(), buffer, size);
    pac.buffer_consumed(size);
    msgpack::object_handle oh;
    if (!pac.next(oh)) {
        cerr << "No MsgPack message in LoLA message." << endl;
        return sensor_frame;
    }
    const auto& map = oh.get().via.map;
    auto* category = map.ptr;
    for (uint32_t i = 0; i < map.size; i++, category++) {
        if (auto ptr = find(sensor_frame_positions, category->key.as<string>())) {
            zip(*ptr, category->val.as<vector<float>>(), [](float* r, float f) { if (r != nullptr) *r = f; });
        }
        else if (auto ptr = find(robotConfig_frame_positions, category->key.as<string>())){
            zip(*ptr, category->val.as<vector<std::string>>(), [](std::string* r, std::string f) { if (r != nullptr) *r = f; });
        }
        else if (auto ptr = find(status_frame_positions, category->key.as<string>())){
            zip(*ptr, category->val.as<vector<int>>(), [](int* r, int f) { if (r != nullptr) *r = f; });
        }
    }
    // These seem to be different between robots but consistent between reboots. Make sure you calibrate each robot otherwise most filters (e.g. Madgwick) won't work correctly!
    // sensor_frame.imu.gyr.yaw *= 1.0764f;
    // sensor_frame.imu.gyr.yaw *= 1.3f;
    // sensor_frame.imu.gyr.pitch *= 1.f;
    // sensor_frame.imu.gyr.roll *= 0.965f;
    return sensor_frame;
}

// pair<char*, size_t> LolaFrameHandler::pack() {
//     buffer.clear();
//     msgpack::packer<msgpack::sbuffer> pk(&buffer);
//     pk.pack_map(actuator_frame_positions.size());
//     for (const auto& kv : actuator_frame_positions) {
//         pk.pack(kv.first);
//         pk.pack_array(kv.second.size());
//         for (float* val : kv.second) {
//             pk.pack(*val);
//         }
//     }
//     return {buffer.data(), buffer.size()};
// }
pair<char*, size_t> LolaFrameHandler::pack() {
    buffer.clear();
    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    pk.pack_map(actuator_frame_positions.size());
    // pk.pack_map(actuator_frame_positions.size() + filter_frame_positions.size());
    for (const auto& kv : actuator_frame_positions) {
        pk.pack(kv.first);
        pk.pack_array(kv.second.size());
        for (float* val : kv.second) {
            pk.pack(*val);
        }
    }
    // add by tl
    // for (const auto& kv2 : filter_frame_positions) {
    //     pk.pack(kv2.first);
    //     pk.pack_array(kv2.second.size());
    //     for (bool* val2 : kv2.second) {
    //         pk.pack(*val2);
    //     }
    // }
    return {buffer.data(), buffer.size()};
}
