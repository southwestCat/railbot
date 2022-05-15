#pragma once

#include "Representations/Sensing/FloatingBaseEstimation.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Blackboard.h"

class FloatingBaseObserverBase
{
public:
    REQUIRES_REPRESENTATION(InertialData);
    REQUIRES_REPRESENTATION(RobotModel);
};

class FloatingBaseObserver : public FloatingBaseObserverBase
{
public:
    void update(FloatingBaseEstimation &f);
    
private:
    void update();
    void estimateOrientation();
    void estimatePosition(FloatingBaseEstimation &f);
    void updateRobot(FloatingBaseEstimation &f);
    sva::PTransform getAnchorFrame();
    void reset(const sva::PTransform &X_o_fb);
    void run(FloatingBaseEstimation &f);
    Vector2f m2rp(Matrix3f m);

    Matrix3f orientation_;
    Vector3f position_;
    float leftFootRatio_ = 0.5f;
};
 