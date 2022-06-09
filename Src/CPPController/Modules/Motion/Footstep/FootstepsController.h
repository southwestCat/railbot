#pragma once

#include "MPC.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Motion/MPCControllerState.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Constants.h"
#include "Tools/Math/Eigen.h"

class FootstepsControllerBase
{
public:
    REQUIRES_REPRESENTATION(FrameInfo);

    USES_REPRESENTATION(MPCControllerState);
};

class FootstepsController : public FootstepsControllerBase
{
public:
    FootstepsController();

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

private:
    std::vector<Eigen::Vector2f> generateFootsteps(float stepLength, float footSpread, int nStepsm, bool leftSwingFirst = true);
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
    void startSwingFoot();
    void startCoMMPCssp();
    void runSingleSupport();
    void runSwingFoot();
    void runCOMMPC();
    void updateMPC(float dsp_duration, float ssp_duration);
    void updateInitialState();
};
