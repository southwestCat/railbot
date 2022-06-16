#pragma once

#include "MPC.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Motion/FootstepControllerState.h"
#include "Representations/Motion/BalanceTarget.h"
#include "Representations/MotionControl/BalanceActionSelection.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/InertialData.h"
#include "Modules/Motion/MotionConfigure.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Constants.h"
#include "Tools/Math/Eigen.h"

#include <fstream>

class FootstepsControllerBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(InertialData);

    USES_REPRESENTATION(FootstepControllerState);
    USES_REPRESENTATION(BalanceActionSelection);
    USES_REPRESENTATION(RobotDimensions);

    MODIFIES_REPRESENTATION(BalanceTarget);
};

class FootstepsController : public FootstepsControllerBase
{
public:
    FootstepsController();
    ~FootstepsController();

    void update(FootstepJointRequest &j);

private:
    void update();

private:
    class WalkingFSM
    {
    public:
        enum FSMState
        {
            Standing,
            DoubleSupport,
            SingleSupport,
            recovery,
            Unknown
        } state;

        WalkingFSM() {}
        WalkingFSM(float ssp_duration, float dsp_duration, float dt)
        {
            this->dt = dt;
            this->ssp_duration = ssp_duration;
            this->dsp_duration = dsp_duration;
            this->next_footstep = 1; //< swing foot
            this->cur_footstep = 0;  //< stance foot
            this->mpc_timestep = round(0.1 / dt) * dt;
            this->state = FSMState::Unknown;
        }

        float ssp_duration;
        float dsp_duration;
        float mpc_timestep;
        float preview_time;
        float rem_time;
        unsigned next_footstep;
        unsigned cur_footstep;
        float dt;
    };

    enum class StanceFoot
    {
        left,
        right
    };

    const float dt = Constants::motionCycleTime;
    float comHeight;
    bool start_walking = false;
    float rem_time;
    float dsp_duration;
    float ssp_duration;
    Eigen::Vector2f stance_target;
    Eigen::Vector2f swing_target;
    MPC x_mpc;
    MPC y_mpc;
    WalkingFSM fsm;
    Point com;
    std::vector<Vector2f> footsteps;
    bool finished = false;
    volatile bool initialState = false;
    JointRequest jointRequest_;
    JointRequest recoveryStartJointRequest_;
    bool updatedJointRequest = false;

    Vector2f hip;            //< hip position in world frame.
    Vector2f hipInitialPos_; //< hip initial position in world frame.
    Vector2f comInitialPos_; //< com initial position in world frame.
    const float hipHeight_ = MotionConfig::hipHeight;
    float STEPHEIGHT_;
    float ANKLEBALANCEOFFSET_;

    unsigned recoveryStartTime_;

    std::ofstream f;

private:
    std::vector<Eigen::Vector2f> generateFootsteps(float stepLength, float footSpread, unsigned nSteps, bool leftSwingFirst = true);
    void startWalking()
    {
        start_walking = true;
        finished = false;
    }
    void exec();
    void startStanding();
    void runStanding();
    void startDoubleSupport();
    void startCoMMPCdsp();
    void runDoubleSupport();
    void startSingleSupport();
    void startCoMMPCssp();
    void runSingleSupport();
    void runCOMMPC();
    void updateMPC(float dsp_duration, float ssp_duration);
    void recoveryToStand();
    void calcJointInDoubleSupport();
    void calcJointInSingleSupport();
    void balance();
    void ankleBalance();
    void leftAnkleBalance();
    void rightAnkleBalance();
    StanceFoot getStanceFoot();

    void setInitialState();
    void test();
};
