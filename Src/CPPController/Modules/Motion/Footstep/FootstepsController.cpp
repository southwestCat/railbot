#include "FootstepsController.h"
#include "Tools/Module/ModuleManager.h"

FootstepsController::FootstepsController()
{
    rem_time = 0.f;
    dsp_duration = round(0.1 / dt) * dt; //< 0.096s
    ssp_duration = round(0.7 / dt) * dt; //< 0.696s
    fsm = WalkingFSM(ssp_duration, dsp_duration, dt);
    startStanding();
}

void FootstepsController::updateInitialState()
{
    static bool once = false;
    if (once)
        return;

    // Vector3f pos = theMPCControllerState->comPosition;
    // Vector3f vel = theMPCControllerState->comVelocity;
    // Vector3f acc = theMPCControllerState->comAcceleration;
    // float stepLength = theMPCControllerState->stepLength;
    // float footSpread = theMPCControllerState->footSpread;
    // float nSteps = theMPCControllerState->nSteps;
    // bool leftSwingFirst = theMPCControllerState->leftSwingFirst;

    Vector3f pos = Vector3f(0.f, 0.f, 260.f);
    Vector3f vel = Vector3f(0.f, 0.f, 0.f);
    Vector3f acc = Vector3f(0.f, 0.f, 0.f);
    float stepLength = 100.f;
    float footSpread = 50.f;
    float nSteps = 1;
    bool leftSwingFirst = true;

    comHeight = pos.z();
    com = Point(pos, vel, acc);
    footsteps = generateFootsteps(stepLength, footSpread, nSteps, leftSwingFirst);

    startStanding();

    once = true;
}

void FootstepsController::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
}

void FootstepsController::update(FootstepJointRequest &j)
{
    update();

    //! return when not in footstep action.
    if (theBalanceActionSelection->targetAction != BalanceActionSelection::footstep)
    {
        initialState = false;
        return;
    }

    //! Initial State
    setInitialState();

    //! Initial JointRequest
    for (int i = 0; i <= Joints::rAnkleRoll; i++)
    {
        j.angles[i] = theBalanceTarget->lastJointRequest.angles[i];
    }

    exec();
}

void FootstepsController::setInitialState()
{
    if (initialState)
    {
        return;
    }

    //! Initial com
    Vector3f comPosition = theFootstepControllerState->comPosition;
    Vector3f comVelocity = theFootstepControllerState->comVelocity;
    comVelocity.y() = 0.f;
    comVelocity.z() = 0.f;
    com = {comPosition, comVelocity};

    //! Set comHeight
    comHeight = comPosition.z();

    //! Generate footsteps
    float stepLength = theFootstepControllerState->stepLength;
    float footSpread = theFootstepControllerState->footSpread;
    unsigned nSteps = theFootstepControllerState->nSteps;
    bool left = theFootstepControllerState->leftSwingFirst;
    footsteps = generateFootsteps(stepLength, footSpread, nSteps, left);

    //! start walking.
    startWalking();

    initialState = true;
}

std::vector<Eigen::Vector2f> FootstepsController::generateFootsteps(float stepLength, float footSpread, unsigned nSteps, bool leftSwingFirst)
{
    std::vector<Vector2f> foots;
    foots.push_back({0.f, +footSpread});
    foots.push_back({0.f, -footSpread});
    if (leftSwingFirst)
    {
        for (unsigned i = 0; i < nSteps; i++)
        {
            foots.push_back({(i + 1) * stepLength, pow(-1, i) * footSpread});
        }
        Vector2f lastStep = foots.at(nSteps + 1);
        foots.push_back({lastStep.x(), -lastStep.y()});
    }
    else
    {
        for (unsigned i = 0; i < nSteps; i++)
        {
            foots.push_back({(i + 1) * stepLength, pow(-1, i + 1) * footSpread});
        }
        Vector2f lastStep = foots.at(nSteps + 1);
        foots.push_back({lastStep.x(), -lastStep.y()});
    }

    return foots;
}

void FootstepsController::exec()
{
    if (fsm.state == WalkingFSM::Standing)
    {
        runStanding();
        printf("[INFO] In Standing.\n");
    }
    else if (fsm.state == WalkingFSM::DoubleSupport)
    {
        runDoubleSupport();
        printf("[INFO] In DoubleSupport.\n");
    }
    else if (fsm.state == WalkingFSM::SingleSupport)
    {
        runSingleSupport();
        printf("[INFO] In SingleSupport.\n");
    }
    else
    {
        printf("[INFO] Unknown state.\n");
    }
}

void FootstepsController::startStanding()
{
    start_walking = false;
    fsm.state = WalkingFSM::Standing;
    runStanding();
}

void FootstepsController::runStanding()
{
    if (start_walking)
    {
        finished = false;
        start_walking = false;
        startDoubleSupport();
    }
    else
    {
        finished = true;
    }
}

void FootstepsController::startDoubleSupport()
{
    fsm.cur_footstep += 1;
    fsm.next_footstep += 1;
    if (fsm.next_footstep == footsteps.size())
    {
        return startStanding();
    }
    // cout << "cur: " << fsm.cur_footstep << " next: " << fsm.next_footstep << endl;
    float dspDuration = fsm.dsp_duration;
    //! The last double support time is increased to ensure robot can stop stably
    if (fsm.next_footstep == footsteps.size() - 1)
    {
        dspDuration = 10 * fsm.dsp_duration;
    }
    stance_target = footsteps[fsm.cur_footstep];
    swing_target = footsteps[fsm.next_footstep];
    fsm.rem_time = dspDuration;
    fsm.state = WalkingFSM::DoubleSupport;
    startCoMMPCdsp();
    runDoubleSupport();
}

void FootstepsController::startCoMMPCdsp()
{
    updateMPC(fsm.rem_time, fsm.ssp_duration);
}

void FootstepsController::runDoubleSupport()
{
    if (fsm.rem_time <= 0.f)
    {
        return startSingleSupport();
    }
    runCOMMPC();
    fsm.rem_time -= dt;
}

void FootstepsController::startSingleSupport()
{
    fsm.rem_time = fsm.ssp_duration;
    fsm.state = WalkingFSM::SingleSupport;
    startSwingFoot();
    startCoMMPCssp();
    return runSingleSupport();
}

void FootstepsController::startSwingFoot()
{
}

void FootstepsController::startCoMMPCssp()
{
    updateMPC(0.f, fsm.rem_time);
}

void FootstepsController::runSingleSupport()
{
    if (fsm.rem_time <= 0.f)
    {
        return startDoubleSupport();
    }
    runSwingFoot();
    runCOMMPC();
    fsm.rem_time -= dt;
}

void FootstepsController::runSwingFoot()
{
}

void FootstepsController::runCOMMPC()
{
    if (fsm.preview_time >= fsm.mpc_timestep)
    {
        if (fsm.state == WalkingFSM::DoubleSupport)
        {
            updateMPC(fsm.rem_time, fsm.ssp_duration);
        }
        else
        {
            updateMPC(0.f, fsm.rem_time);
        }
    }
    // Vector3f com_jerk = {0, 0, 0};
    float xjerk = x_mpc.result()[0];
    float yjerk = y_mpc.result()[0];
    Vector3f com_jerk = {xjerk, yjerk, 0};
    com.integrateConstantJerk(com_jerk, dt);

    // cout << "comjerk: " << com_jerk.transpose() << endl;
    // cout << "com:     " << com.x() << " " << com.y() << endl;

    fsm.preview_time += dt;
}

void FootstepsController::updateMPC(float dsp_duration, float ssp_duration)
{
    int nb_preview_steps = 16;
    float T = fsm.mpc_timestep;
    int nb_init_dsp_steps = (int)round(dsp_duration / T);
    int nb_init_ssp_steps = (int)round(ssp_duration / T);
    int nb_dsp_steps = (int)round(fsm.dsp_duration / T);
    Matrix3f A;
    A << 1.f, T, T * T / 2.f,
        0.f, 1.f, T,
        0.f, 0.f, 1.f;
    Vector3f B;
    B << T * T * T / 6.f, T * T / 2.f, T;
    float h = comHeight;
    float g = Constants::g;
    Eigen::Matrix<float, 2, 3> C;
    C << 1.f, 0.f, -h / g, -1.f, 0.f, h / g;
    Eigen::Matrix<float, 1, 1> D;
    D = Eigen::Matrix<float, 1, 1>::Zero();
    std::vector<Vector2f> e[2];

    //! 0:x 1:y
    for (int coord = 0; coord < 2; coord++)
    {
        float maxX = 60.f;
        float minX = -60.f;
        float maxY = 30.f;
        float minY = -30.f;

        float cur_max_x = stance_target.x() + maxX;
        float cur_max_y = stance_target.y() + maxY;
        float cur_min_x = stance_target.x() + minX;
        float cur_min_y = stance_target.y() + minY;

        float next_max_x = swing_target.x() + maxX;
        float next_max_y = swing_target.y() + maxY;
        float next_min_x = swing_target.x() + minX;
        float next_min_y = swing_target.y() + minY;

        float cur_max = coord == 0 ? cur_max_x : cur_max_y;
        float cur_min = coord == 0 ? cur_min_x : cur_min_y;
        float next_max = coord == 0 ? next_max_x : next_max_y;
        float next_min = coord == 0 ? next_min_x : next_min_y;

        for (int i = 0; i < nb_preview_steps; i++)
        {
            if (i < nb_init_dsp_steps)
            {
                e[coord].push_back({cur_max + 100.f, -cur_min + 100.f});
            }
            else if (i - nb_init_dsp_steps <= nb_init_ssp_steps)
            {
                e[coord].push_back({+cur_max, -cur_min});
            }
            else if (i - nb_init_dsp_steps - nb_init_ssp_steps < nb_dsp_steps)
            {
                e[coord].push_back({+next_max + 100.f, -next_min + 100.f});
            }
            else
            {
                e[coord].push_back({+next_max, -next_min});
            }
        }
    }

    Vector3f x_init;
    x_init << com.x(), com.xd(), com.xdd();
    Vector3f x_goal;
    x_goal << swing_target.x(), 0, 0;
    x_mpc = MPC(A, B, C, D, e[0], x_init, x_goal, nb_preview_steps, 10.f, 1.f, 1.f);
    Vector3f y_init;
    y_init << com.y(), com.yd(), com.ydd();
    Vector3f y_goal;
    y_goal << swing_target.y(), 0, 0;
    y_mpc = MPC(A, B, C, D, e[1], y_init, y_goal, nb_preview_steps, 10.f, 1.f, 1.f);

    // cout << "next step: " << fsm.next_footstep << endl;
    // cout << "x goal:" << endl;
    // cout << x_goal.transpose() << endl;
    // cout << "y goal:" << endl;
    // cout << y_goal.transpose() << endl;

    x_mpc.solve();
    y_mpc.solve();
    fsm.preview_time = 0.f;
}
