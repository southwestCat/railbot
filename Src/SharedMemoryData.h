#pragma once

#define SEM_NAME "/railbot_shared_sem"
#define MEM_NAME "/railbot_shared_mem"

enum SensorIds
{
    // joint sensors
    headYawPositionSensor,
    headYawCurrentSensor,
    headYawTemperatureSensor,
    headPitchPositionSensor,
    headPitchCurrentSensor,
    headPitchTemperatureSensor,
    lShoulderPitchPositionSensor,
    lShoulderPitchCurrentSensor,
    lShoulderPitchTemperatureSensor,
    lShoulderRollPositionSensor,
    lShoulderRollCurrentSensor,
    lShoulderRollTemperatureSensor,
    lElbowYawPositionSensor,
    lElbowYawCurrentSensor,
    lElbowYawTemperatureSensor,
    lElbowRollPositionSensor,
    lElbowRollCurrentSensor,
    lElbowRollTemperatureSensor,
    lWristYawPositionSensor,
    lWristYawCurrentSensor,
    lWristYawTemperaturSensor,
    lHandPositionSensor,
    lHandCurrentSensor,
    lHandTemperaturSensor,
    rShoulderPitchPositionSensor,
    rShoulderPitchCurrentSensor,
    rShoulderPitchTemperatureSensor,
    rShoulderRollPositionSensor,
    rShoulderRollCurrentSensor,
    rShoulderRollTemperatureSensor,
    rElbowYawPositionSensor,
    rElbowYawCurrentSensor,
    rElbowYawTemperatureSensor,
    rElbowRollPositionSensor,
    rElbowRollCurrentSensor,
    rElbowRollTemperatureSensor,
    rWristYawPositionSensor,
    rWristYawCurrentSensor,
    rWristYawTemperaturSensor,
    rHandPositionSensor,
    rHandCurrentSensor,
    rHandTemperaturSensor,
    lHipYawPitchPositionSensor,
    lHipYawPitchCurrentSensor,
    lHipYawPitchTemperatureSensor,
    lHipRollPositionSensor,
    lHipRollCurrentSensor,
    lHipRollTemperatureSensor,
    lHipPitchPositionSensor,
    lHipPitchCurrentSensor,
    lHipPitchTemperatureSensor,
    lKneePitchPositionSensor,
    lKneePitchCurrentSensor,
    lKneePitchTemperatureSensor,
    lAnklePitchPositionSensor,
    lAnklePitchCurrentSensor,
    lAnklePitchTemperatureSensor,
    lAnkleRollPositionSensor,
    lAnkleRollCurrentSensor,
    lAnkleRollTemperatureSensor,
    rHipRollPositionSensor,
    rHipRollCurrentSensor,
    rHipRollTemperatureSensor,
    rHipPitchPositionSensor,
    rHipPitchCurrentSensor,
    rHipPitchTemperatureSensor,
    rKneePitchPositionSensor,
    rKneePitchCurrentSensor,
    rKneePitchTemperatureSensor,
    rAnklePitchPositionSensor,
    rAnklePitchCurrentSensor,
    rAnklePitchTemperatureSensor,
    rAnkleRollPositionSensor,
    rAnkleRollCurrentSensor,
    rAnkleRollTemperatureSensor,

    // touch sensors
    headTouchFrontSensor,
    headTouchMiddleSensor,
    headTouchRearSensor,
    lHandTouchBackSensor,
    lHandTouchLeftSensor,
    lHandTouchRightSensor,
    rHandTouchBackSensor,
    rHandTouchLeftSensor,
    rHandTouchRightSensor,

    // switches
    lBumperLeftSensor,
    lBumperRightSensor,
    rBumperLeftSensor,
    rBumperRightSensor,
    chestButtonSensor,

    // inertial sensors
    gyroXSensor,
    gyroYSensor,
    gyroZSensor,
    accXSensor,
    accYSensor,
    accZSensor,
    angleXSensor,
    angleYSensor,
    angleZSensor,

    // battery sensors
    batteryCurrentSensor,
    batteryChargeSensor,
    batteryStatusSensor,
    batteryTemperatureSensor,

    // fsr sensors
    lFSRFrontLeftSensor,
    lFSRFrontRightSensor,
    lFSRRearLeftSensor,
    lFSRRearRightSensor,
    rFSRFrontLeftSensor,
    rFSRFrontRightSensor,
    rFSRRearLeftSensor,
    rFSRRearRightSensor,
    lFSRTotalSensor,
    rFSRTotalSensor,

    mumOfSensorIds
};

enum ActuatorIds
{
    // joint request
    headYawPositionActuator,
    headPitchPositionActuator,
    lShoulderPitchPositionActuator,
    lShoulderRollPositionActuator,
    lElbowYawPositionActuator,
    lElbowRollPositionActuator,
    lWristYawPositionActuator,
    lHandPositionActuator,
    rShoulderPitchPositionActuator,
    rShoulderRollPositionActuator,
    rElbowYawPositionActuator,
    rElbowRollPositionActuator,
    rWristYawPositionActuator,
    rHandPositionActuator,
    lHipYawPitchPositionActuator,
    lHipRollPositionActuator,
    lHipPitchPositionActuator,
    lKneePitchPositionActuator,
    lAnklePitchPositionActuator,
    lAnkleRollPositionActuator,
    rHipRollPositionActuator,
    rHipPitchPositionActuator,
    rKneePitchPositionActuator,
    rAnklePitchPositionActuator,
    rAnkleRollPositionActuator,
    numOfPositionActuatorIds,

    // stiffness request
    headYawStiffnessActuator = numOfPositionActuatorIds,
    headPitchStiffnessActuator,
    lShoulderPitchStiffnessActuator,
    lShoulderRollStiffnessActuator,
    lElbowYawStiffnessActuator,
    lElbowRollStiffnessActuator,
    lWristYawStiffnessActuator,
    lHandStiffnessActuator,
    rShoulderPitchStiffnessActuator,
    rShoulderRollStiffnessActuator,
    rElbowYawStiffnessActuator,
    rElbowRollStiffnessActuator,
    rWristYawStiffnessActuator,
    rHandStiffnessActuator,
    lHipYawPitchStiffnessActuator,
    lHipRollStiffnessActuator,
    lHipPitchStiffnessActuator,
    lKneePitchStiffnessActuator,
    lAnklePitchStiffnessActuator,
    lAnkleRollStiffnessActuator,
    rHipRollStiffnessActuator,
    rHipPitchStiffnessActuator,
    rKneePitchStiffnessActuator,
    rAnklePitchStiffnessActuator,
    rAnkleRollStiffnessActuator,

    // led request
    faceLedRedLeft0DegActuator,
    faceLedRedLeft45DegActuator,
    faceLedRedLeft90DegActuator,
    faceLedRedLeft135DegActuator,
    faceLedRedLeft180DegActuator,
    faceLedRedLeft225DegActuator,
    faceLedRedLeft270DegActuator,
    faceLedRedLeft315DegActuator,
    faceLedGreenLeft0DegActuator,
    faceLedGreenLeft45DegActuator,
    faceLedGreenLeft90DegActuator,
    faceLedGreenLeft135DegActuator,
    faceLedGreenLeft180DegActuator,
    faceLedGreenLeft225DegActuator,
    faceLedGreenLeft270DegActuator,
    faceLedGreenLeft315DegActuator,
    faceLedBlueLeft0DegActuator,
    faceLedBlueLeft45DegActuator,
    faceLedBlueLeft90DegActuator,
    faceLedBlueLeft135DegActuator,
    faceLedBlueLeft180DegActuator,
    faceLedBlueLeft225DegActuator,
    faceLedBlueLeft270DegActuator,
    faceLedBlueLeft315DegActuator,
    faceLedRedRight0DegActuator,
    faceLedRedRight45DegActuator,
    faceLedRedRight90DegActuator,
    faceLedRedRight135DegActuator,
    faceLedRedRight180DegActuator,
    faceLedRedRight225DegActuator,
    faceLedRedRight270DegActuator,
    faceLedRedRight315DegActuator,
    faceLedGreenRight0DegActuator,
    faceLedGreenRight45DegActuator,
    faceLedGreenRight90DegActuator,
    faceLedGreenRight135DegActuator,
    faceLedGreenRight180DegActuator,
    faceLedGreenRight225DegActuator,
    faceLedGreenRight270DegActuator,
    faceLedGreenRight315DegActuator,
    faceLedBlueRight0DegActuator,
    faceLedBlueRight45DegActuator,
    faceLedBlueRight90DegActuator,
    faceLedBlueRight135DegActuator,
    faceLedBlueRight180DegActuator,
    faceLedBlueRight225DegActuator,
    faceLedBlueRight270DegActuator,
    faceLedBlueRight315DegActuator,
    earsLedLeft0DegActuator,
    earsLedLeft36DegActuator,
    earsLedLeft72DegActuator,
    earsLedLeft108DegActuator,
    earsLedLeft144DegActuator,
    earsLedLeft180DegActuator,
    earsLedLeft216DegActuator,
    earsLedLeft252DegActuator,
    earsLedLeft288DegActuator,
    earsLedLeft324DegActuator,
    earsLedRight0DegActuator,
    earsLedRight36DegActuator,
    earsLedRight72DegActuator,
    earsLedRight108DegActuator,
    earsLedRight144DegActuator,
    earsLedRight180DegActuator,
    earsLedRight216DegActuator,
    earsLedRight252DegActuator,
    earsLedRight288DegActuator,
    earsLedRight324DegActuator,
    chestBoardLedRedActuator,
    chestBoardLedGreenActuator,
    chestBoardLedBlueActuator,
    headLedRearLeft0Actuator,
    headLedRearLeft1Actuator,
    headLedRearLeft2Actuator,
    headLedRearRight0Actuator,
    headLedRearRight1Actuator,
    headLedRearRight2Actuator,
    headLedMiddleRight0Actuator,
    headLedFrontRight0Actuator,
    headLedFrontRight1Actuator,
    headLedFrontLeft0Actuator,
    headLedFrontLeft1Actuator,
    headLedMiddleLeft0Actuator,
    lFootLedRedActuator,
    lFootLedGreenActuator,
    lFootLedBlueActuator,
    rFootLedRedActuator,
    rFootLedGreenActuator,
    rFootLedBlueActuator,

    numOfActuatorIds
};

const int numOfStiffnessActuatorIds = numOfPositionActuatorIds;
const int numOfLedActuatorIds = rFootLedBlueActuator + 1 - faceLedRedLeft0DegActuator;
const int numOfDifSensors = 3;

struct SharedData
{
    volatile int readingSensors;
    volatile int newestSensors;
    volatile int readingActuators;
    volatile int newestActuators;

    volatile int state = -1;

    float sensors[3][mumOfSensorIds];
    float actuators[3][numOfActuatorIds];
};