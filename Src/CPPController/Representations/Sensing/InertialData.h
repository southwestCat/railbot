#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"

/**
 * The inertialData contains filtered data from the IMU.
 */
class InertialData : public InertialSensorData
{
public:
    Quaternionf orientation2D = Quaternionf::Identity(); /** The orientation of the torso represented as a quaternion without the z-Rotation. */
    Quaternionf orientation3D = Quaternionf::Identity(); /** The orientation of the torso represented as a quaternion including the z-Rotation. */
    Vector3f filteredAcc = Vector3f::Identity();
};