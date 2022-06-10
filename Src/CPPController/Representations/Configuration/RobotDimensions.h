#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/FsrSensors.h"

class RobotDimensions
{
public:
    RobotDimensions()
    {
        leftFsrPositions[FsrSensors::fl] = Vector2f(70.25f, 29.9f);
        leftFsrPositions[FsrSensors::fr] = Vector2f(70.25f, -23.1f);
        leftFsrPositions[FsrSensors::bl] = Vector2f(-30.25f, 29.9f);
        leftFsrPositions[FsrSensors::br] = Vector2f(-29.65f, -19.1f);

        rightFsrPositions[FsrSensors::fl] = Vector2f(70.25f, 23.1f);
        rightFsrPositions[FsrSensors::fr] = Vector2f(70.25f, -29.9f);
        rightFsrPositions[FsrSensors::bl] = Vector2f(-29.65f, 19.1f);
        rightFsrPositions[FsrSensors::br] = Vector2f(-30.25f, -29.9f);
    }

    float getXOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? xOffsetNeckToLowerCamera : xOffsetNeckToUpperCamera; }

    float getZOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? zOffsetNeckToLowerCamera : zOffsetNeckToUpperCamera; }

    Angle getTiltNeckToCamera(bool lowerCamera) const { return lowerCamera ? tiltNeckToLowerCamera : tiltNeckToUpperCamera; }

    float yHipOffset = 50.f;
    float upperLegLength = 100.f;
    float lowerLegLength = 102.9f;
    float footHeight = 45.19f;
    float footLength = 110.f;
    float soleToFrontEdgeLength = 110.f;
    float soleToInnerEdgeLength = 38.78f;
    float soleToOuterEdgeLength = 50.02f;
    float soleToBackEdgeLength = 50.f;
    float hipToNeckLength = 211.5f;
    float halfSoleLength = 80.f;
    float halfSoleWidth = 44.4f;
    float soleFriction = 0.7f;
    Vector2f leftAnkleToSoleCenter = {30.f, 5.62f};
    Vector2f rightAnkleToSoleCenter = {30.f, -5.62f};

    float xOffsetNeckToLowerCamera = 50.71f;
    float zOffsetNeckToLowerCamera = 17.74f;
    Angle tiltNeckToLowerCamera = 39.7_deg;

    float xOffsetNeckToUpperCamera = 58.71f;
    float zOffsetNeckToUpperCamera = 63.64;
    Angle tiltNeckToUpperCamera = 1.2_deg;

    Vector3f armOffset = Vector3f(0.f, 98.f, 185.f);
    float yOffsetElbowToShoulder = 15.f;
    float upperArmLength = 105.f;
    float lowerArmLength = 130.f;
    float xOffsetElbowToWrist = 55.95f;
    Vector3f handOffset = Vector3f(57.75f, 0.f, 12.31f);
    float handRadius = 32.5f;
    float armRadius = 25.f;

    Vector3f imuOffset = Vector3f(-8.f, 6.06f, 112.f);

    std::array<Vector2f, FsrSensors::numOfFsrSensors> leftFsrPositions;
    std::array<Vector2f, FsrSensors::numOfFsrSensors> rightFsrPositions;
};
