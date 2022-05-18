#include "InertialDataProvider.h"
#include "Tools/Math/Constants.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Module/ModuleManager.h"

#include <iostream>
#include <cmath>

InertialDataProvider::InertialDataProvider()
{
    beta = 0.1f;
    q0 = 1.f;
    q1 = 0.f;
    q2 = 0.f;
    q3 = 0.f;
}

void InertialDataProvider::update()
{
    UPDATE_REPRESENTATION(InertialSensorData);
    UPDATE_REPRESENTATION(IMUCalibration);

    // float ax = theInertialSensorData->acc.x();
    // float ay = theInertialSensorData->acc.y();
    // float az = theInertialSensorData->acc.z();
    // printf("acc: %3.3f %3.3f %3.3f \n\n", ax, ay, az);

    // float gx = theInertialSensorData->gyro.x();
    // float gy = theInertialSensorData->gyro.y();
    // float gz = theInertialSensorData->gyro.z();
    // printf("gyro: %3.3f %3.3f %3.3f \n\n", gx, gy, gz);
}

void InertialDataProvider::update(InertialData &inertialData)
{
    update();

    float gx, gy, gz;
    float ax, ay, az;
    gx = theInertialSensorData->gyro.x();
    gy = theInertialSensorData->gyro.y();
    gz = theInertialSensorData->gyro.z();
    ax = (theInertialSensorData->acc.x() - theIMUCalibration->accBias.x()) * theIMUCalibration->accFactor.x();
    ay = (theInertialSensorData->acc.y() - theIMUCalibration->accBias.y()) * theIMUCalibration->accFactor.y();
    az = (theInertialSensorData->acc.z() - theIMUCalibration->accBias.z()) * theIMUCalibration->accFactor.z();

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _4q3, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = 1.f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _4q3 = 4.0f * q3;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q0q0 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
        s2 = _4q2 * q0q0 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
        s3 = _4q3 * q1q1 + _2q1 * ax + _4q3 * q2q2 + _2q2 * ay;
        recipNorm = 1.f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * Constants::motionCycleTime;
    q1 += qDot2 * Constants::motionCycleTime;
    q2 += qDot3 * Constants::motionCycleTime;
    q3 += qDot4 * Constants::motionCycleTime;

    // Normalise quaternion
    recipNorm = 1.f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    inertialData.orientation3D.w() = q0;
    inertialData.orientation3D.x() = q1;
    inertialData.orientation3D.y() = q2;
    inertialData.orientation3D.z() = q3;
    inertialData.orientation2D = Rotation::removeZRotation(inertialData.orientation3D);

    float roll, pitch;
    // Eigen::Matrix3f t_m = inertialData.orientation2D.matrix();
    Eigen::Vector3f g_v(0.f, 0.f, -1.f);
    // Eigen::Vector3f acc_v = t_m.inverse() * g_v;
    Eigen::Vector3f acc_v = inertialData.orientation2D.conjugate() * g_v;
    if (acc_v.z() <= 0.f)
    {
        roll = atan2(-acc_v.y(), sqrt(acc_v.x() * acc_v.x() + acc_v.z() * acc_v.z()));
        pitch = atan2(acc_v.x(), sqrt(acc_v.y() * acc_v.y() + acc_v.z() * acc_v.z()));
    }
    else
    {
        roll = atan2(-acc_v.y(), -sqrt(acc_v.x() * acc_v.x() + acc_v.z() * acc_v.z()));
        pitch = atan2(acc_v.x(), -sqrt(acc_v.y() * acc_v.y() + acc_v.z() * acc_v.z()));
    }

    inertialData.angle << roll, pitch, 0.f;
}
