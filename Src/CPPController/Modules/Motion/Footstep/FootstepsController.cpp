#include "FootstepsController.h"
#include "SpaceVecAlg/PTransform.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Module/ModuleManager.h"

FootstepsController::FootstepsController()
{
    rem_time = 0.f;
    dsp_duration = round(0.1 / dt) * dt; //< 0.096s
    ssp_duration = round(0.7 / dt) * dt; //< 0.696s
    fsm = WalkingFSM(ssp_duration, dsp_duration, dt);
    startStanding();
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
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        j.angles[i] = theBalanceTarget->lastJointRequest.angles[i];
    }

    exec();
    // test();

    //! update request
    // updatedJointRequest = false;
    if (!updatedJointRequest)
        return;
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
    {
        j.angles[i] = jointRequest_.angles[i];
    }
    updatedJointRequest = false;
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
    comInitialPos_ = {comPosition.x(), comPosition.y()};

    //! Set comHeight
    comHeight = comPosition.z();

    //! Set step height
    STEPHEIGHT_ = 15.f;

    //! Calculate initial hip position
    float hipInitialX = theFootstepControllerState->hipPosition.x();
    float hipInitialY = theFootstepControllerState->hipPosition.y();
    hipInitialPos_ = {hipInitialX, hipInitialY};

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

    float surfaceSpread = footSpread + theRobotDimensions->leftAnkleToSoleCenter.y();

    if (leftSwingFirst)
    {
        foots.push_back({0.f, +surfaceSpread});
        foots.push_back({0.f, -surfaceSpread});
        for (unsigned i = 0; i < nSteps; i++)
        {
            foots.push_back({(i + 1) * stepLength, pow(-1, i) * surfaceSpread});
        }
        Vector2f lastStep = foots.at(nSteps + 1);
        foots.push_back({lastStep.x(), -lastStep.y()});
    }
    else
    {
        foots.push_back({0.f, -surfaceSpread});
        foots.push_back({0.f, +surfaceSpread});
        for (unsigned i = 0; i < nSteps; i++)
        {
            foots.push_back({(i + 1) * stepLength, pow(-1, i + 1) * surfaceSpread});
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
    }
    else if (fsm.state == WalkingFSM::DoubleSupport)
    {
        runDoubleSupport();
    }
    else if (fsm.state == WalkingFSM::SingleSupport)
    {
        runSingleSupport();
    }
    else if (fsm.state == WalkingFSM::recovery)
    {
        recoveryToStand();
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
        fsm.state = WalkingFSM::recovery;
        recoveryStartTime_ = theFrameInfo->time;
        return recoveryToStand();
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
    printf("DoubleSupport.\n");
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
    calcJointInDoubleSupport();
    fsm.rem_time -= dt;
}

void FootstepsController::calcJointInDoubleSupport()
{
    bool left = theFootstepControllerState->leftSwingFirst;

    Pose3f targetL;
    Pose3f targetR;
    sva::PTransform OTL;
    sva::PTransform OTR;
    //! WTO
    hip = hipInitialPos_ + Vector2f(com.x(), com.y()) - comInitialPos_;

    Vector3f WPO = {hip.x(), hip.y(), hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    if (left) //< Left swing first
    {
        if (fsm.cur_footstep % 2 == 1) //< Right stance cur_footsteps
        {
            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};

            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep - 1).x(), footsteps.at(fsm.cur_footstep - 1).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};
        }
        else //< left stance cur_footsteps
        {
            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! targer L
            targetL = {OTL.rotation(), OTL.translation()};

            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep - 1).x(), footsteps.at(fsm.cur_footstep - 1).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};
        }
    }
    else //< Right swing first
    {
        if (fsm.cur_footstep % 2 == 1) //< Left stance cur_footsteps
        {
            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};

            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep - 1).x(), footsteps.at(fsm.cur_footstep - 1).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};
        }
        else //< Right stance cur_footsteps
        {
            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};

            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep - 1).x(), footsteps.at(fsm.cur_footstep - 1).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};
        }
    }

    //! Calculate jointRequest.
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;
}

void FootstepsController::startSingleSupport()
{
    fsm.rem_time = fsm.ssp_duration;
    fsm.state = WalkingFSM::SingleSupport;
    startCoMMPCssp();
    printf("SingleSupport.\n");
    return runSingleSupport();
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
    runCOMMPC();
    calcJointInSingleSupport();
    fsm.rem_time -= dt;
}

void FootstepsController::calcJointInSingleSupport()
{
    bool left = theFootstepControllerState->leftSwingFirst;

    Pose3f targetL;
    Pose3f targetR;
    sva::PTransform OTL;
    sva::PTransform OTR;
    //! WTO
    hip = hipInitialPos_ + Vector2f(com.x(), com.y()) - comInitialPos_;
    Vector3f WPO = {hip.x(), hip.y(), hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    //! Cycloid swing foot path
    float ratio = fsm.rem_time / fsm.ssp_duration;
    float theta = ratio * 2 * pi;
    float a = theFootstepControllerState->stepLength / 2 / pi;
    float x = a * (theta - sin(theta));                       //< footsteps.x() += x
    float z = a * (1.f - cos(theta)) / 2.f / a * STEPHEIGHT_; //< footsteps.z() += z

    if (left) //< Left swing first
    {
        if (fsm.cur_footstep % 2 == 1) //< left swing
        {
            //! WTL
            Vector3f WPL = {footsteps.at(fsm.next_footstep).x() - x, footsteps.at(fsm.next_footstep).y(), z};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};

            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};
        }
        else //< right swing
        {
            //! WTR
            Vector3f WPR = {footsteps.at(fsm.next_footstep).x() - x, footsteps.at(fsm.next_footstep).y(), z};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};

            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};
        }
    }
    else //< Right swing first
    {
        if (fsm.cur_footstep % 2 == 1) //< right swing
        {
            //! WTR
            Vector3f WPR = {footsteps.at(fsm.next_footstep).x() - x, footsteps.at(fsm.next_footstep).y(), z};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};

            //! WTL
            Vector3f WPL = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};
        }
        else //< left swing
        {
            //! WTL
            Vector3f WPL = {footsteps.at(fsm.next_footstep).x() - x, footsteps.at(fsm.next_footstep).y(), z};
            Matrix3f WRL = Matrix3f::Identity();
            sva::PTransform WTL = {WRL, WPL};
            //! LTSL
            Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
            Matrix3f LRSL = Matrix3f::Identity();
            sva::PTransform LTSL = {LRSL, LPSL};
            //! OTL
            sva::PTransform OTL = WTO.inv() * WTL * LTSL;
            //! target L
            targetL = {OTL.rotation(), OTL.translation()};

            //! WTR
            Vector3f WPR = {footsteps.at(fsm.cur_footstep).x(), footsteps.at(fsm.cur_footstep).y(), 0.f};
            Matrix3f WRR = Matrix3f::Identity();
            sva::PTransform WTR = {WRR, WPR};
            //! RTSR
            Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
            Matrix3f RRSR = Matrix3f::Identity();
            sva::PTransform RTSR = {RRSR, RPSR};
            //! OTR
            sva::PTransform OTR = WTO.inv() * WTR * RTSR;
            //! target R
            targetR = {OTR.rotation(), OTR.translation()};
        }
    }

    //! Calculate jointRequest.
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;
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

void FootstepsController::recoveryToStand()
{
    float t = (float)(theFrameInfo->time - recoveryStartTime_) / 1000.f;
    //! return to initial standing.
    if (t > 1.f)
    {
        return startStanding();
    }
}

void FootstepsController::test()
{
    const float x = hipInitialPos_.x();
    const float y = hipInitialPos_.y();
    static float t = 0.f;
    float yOffset = com.y() - comInitialPos_.y();

    //! WTO: bhuman frame in world frame.
    // Vector3f WPO = {x, y + yOffset, hipHeight_};
    Vector3f WPO = {x, y, hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    //! WTL: left contact surface frame in world frame.
    float a = 80.f / 2.f / pi;
    const float T = 1.f;
    float theta = t < T ? t / T * 2 * pi : 2 * pi;
    float WTL_x = a * (theta - sin(theta));
    float WTL_y = 55.62f;
    float WTL_z = (1 - cos(theta)) * 5.f;
    Vector3f WPL = {WTL_x, WTL_y, WTL_z};
    Matrix3f WRL = Matrix3f::Identity();
    sva::PTransform WTL = {WRL, WPL};
    //! LTSL: sole left frame in left contact surface frame.
    Vector3f LPSL = {-theRobotDimensions->leftAnkleToSoleCenter.x(), -theRobotDimensions->leftAnkleToSoleCenter.y(), 0.f};
    Matrix3f LRSL = Matrix3f::Identity();
    sva::PTransform LTSL = {LRSL, LPSL};
    //! OTSL: sole left frame in bhuman frame.
    sva::PTransform OTSL = WTO.inv() * WTL * LTSL;
    Pose3f targetL = {OTSL.rotation(), OTSL.translation()};

    //! WTR: right contact surface frame in world frame.
    Vector3f WPR = {0.f, -55.62f, 0.f};
    Matrix3f WRR = Matrix3f::Identity();
    sva::PTransform WTR = {WRR, WPR};
    //! RTSR: sole right frame in right contact surface frame.
    Vector3f RPSR = {-theRobotDimensions->rightAnkleToSoleCenter.x(), -theRobotDimensions->rightAnkleToSoleCenter.y(), 0.f};
    Matrix3f RRSR = Matrix3f::Identity();
    sva::PTransform RTSR = {RRSR, RPSR};
    //! OTSR: sole right frame in bhuman frame.
    sva::PTransform OTSR = WTO.inv() * WTR * RTSR;
    Pose3f targetR = {OTSR.rotation(), OTSR.translation()};

    //! Calculate jointRequest.
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;

    //! update time.
    t += dt;
}
