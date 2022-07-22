#include "FootstepsController.h"
#include "SpaceVecAlg/PTransform.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Module/ModuleManager.h"

FootstepsController::FootstepsController()
{
    //! Configuration
    rem_time = 0.f;
    dsp_duration = round(0.1 / dt) * dt; //< 0.096s
    ssp_duration = round(0.4 / dt) * dt; //< 0.696s
    fsm = WalkingFSM(ssp_duration, dsp_duration, dt);
    startStanding();

    // flog.open("log.txt");
    // fcom.open("com.txt");
}

FootstepsController::~FootstepsController()
{
    // flog.close();
    // fcom.close();
}

void FootstepsController::update()
{
    UPDATE_REPRESENTATION(FrameInfo);
    UPDATE_REPRESENTATION(InertialData);
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
    // balance();
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
    comPosition.y() = 0.f;
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

    //! Initial fsm
    fsm.cur_footstep = 0;
    fsm.next_footstep = 1;

    //! Clear footsteps
    if (!footsteps.empty())
    {
        footsteps.clear();
    }

    //! Generate footsteps
    float stepLength = theFootstepControllerState->stepLength;
    float footSpread = theFootstepControllerState->footSpread;
    unsigned nSteps = theFootstepControllerState->nSteps;
    bool left = theFootstepControllerState->leftSwingFirst;
    swingLeftFirst = left;
    footsteps = generateFootsteps(stepLength, footSpread, nSteps, left);

    //! Print information.
    printf(">\n");
    if (stepLength > 0)
    {
        printf("Forward.\n");
        // fcom << "Forward.\n";
    }
    else
    {
        printf("Backward.\n");
        // fcom << "Backward.\n";
    }
    if (left)
    {
        printf("left foot first.\n");
        // fcom << "left foot first.\n";
    }
    else
    {
        printf("right foot first.\n");
        // fcom << "right foot first.\n";
    }
    for (auto f : footsteps)
    {
        printf("%3.3f, %3.3f\n", f.x(), f.y());
        // fcom << f.x() << " " << f.y() << std::endl;
    }
    printf("----\n\n");
    // fcom << "----\n\n";

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
    else if (fsm.state == WalkingFSM::standRebalance)
    {
        standRebalance();
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
        //! Update isFootstepsControlDone
        theBalanceTarget->isFootstepsControlDone = false;

        finished = false;
        start_walking = false;
        startDoubleSupport();
    }
    else
    {
        finished = true;
        theBalanceTarget->isFootstepsControlDone = true;
    }
}

void FootstepsController::startDoubleSupport()
{
    fsm.cur_footstep += 1;
    fsm.next_footstep += 1;
    if (fsm.next_footstep == footsteps.size())
    {
        fsm.state = WalkingFSM::standRebalance;
        standRebalanceZ = 0.f;
        return standRebalance();
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
    calcJointInDoubleSupport();
    fsm.rem_time -= dt;
}

void FootstepsController::calcJointInDoubleSupport()
{
    bool left = swingLeftFirst;

    Pose3f targetL;
    Pose3f targetR;
    sva::PTransform OTL;
    sva::PTransform OTR;
    //! WTO
    hip = hipInitialPos_ + Vector2f(com.x(), com.y()) - comInitialPos_;

    Vector3f WPO = {hip.x(), hip.y(), hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    //! Printf info
    // printf("%3.3f %3.3f %d\n", WPO.x(), WPO.y(), 0);

    //! LOG fcom
    // fcom << "[" << theFrameInfo->time << "]" << " D: " << hip.x() << " " << hip.y() << " " << std::endl;

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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "DS: Left swing first, right stance. \n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "DS: Left swing first, left stance.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "DS: Right swing first, left stance.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " <<WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "DS: Right swing first, right stance.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
        }
    }

    //! Calculate jointRequest.
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    // printf("DoubleSupport>\n");
    // printf("targetL: %f %f %f\n", targetL.translation.x(), targetL.translation.y(), targetL.translation.z());
    // printf("targetR: %f %f %f\n", targetR.translation.x(), targetR.translation.y(), targetR.translation.z());
    // printf("----\n\n");


    //! Update BalanceTarget
    if (isPossible)
    {
        theBalanceTarget->lastJointRequest = jointRequest_;
        theBalanceTarget->soleLeftRequest = targetL;
        theBalanceTarget->soleRightRequest = targetR;
    }
    else
    {
        printf("[WARNING]: In calcJointInDoubleSupport(), foot target cannot reach.\n");
    }
}

void FootstepsController::startSingleSupport()
{
    fsm.rem_time = fsm.ssp_duration;
    fsm.state = WalkingFSM::SingleSupport;
    startCoMMPCssp();
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
    bool left = swingLeftFirst;

    Pose3f targetL;
    Pose3f targetR;
    sva::PTransform OTL;
    sva::PTransform OTR;
    //! WTO
    hip = hipInitialPos_ + Vector2f(com.x(), com.y()) - comInitialPos_;
    Vector3f WPO = {hip.x(), hip.y(), hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    //! Print info
    // printf("%3.3f %3.3f %d\n", WPO.x(), WPO.y(), 1);
    // f << com.x() << " " << com.y() << " " << 1 << std::endl;

    //! LOG fcom
    // fcom << "[" << theFrameInfo->time << "]" << " S: " << com.x() << " " << com.y() << " " << std::endl;

    const float KICKDOWN_TIME = 0.1f;
    const float KICKDOWN_DEPTH = 0.2f;
    float remTime = fsm.ssp_duration - fsm.rem_time;
    float x, z;
    if (remTime < KICKDOWN_TIME)
    {
        x = 2 * theFootstepControllerState->stepLength;
        if (fsm.cur_footstep == 1 || fsm.next_footstep == footsteps.size() - 1)
        {
            x = theFootstepControllerState->stepLength;
        }
        z = -KICKDOWN_DEPTH * sin(2.f * pi * remTime / KICKDOWN_TIME);
    }
    else
    {
        //! Cycloid swing foot path
        float ratio = fsm.rem_time / (fsm.ssp_duration - KICKDOWN_TIME);
        float theta = ratio * 2 * pi;
        float a = 2 * theFootstepControllerState->stepLength / 2 / pi;
        if (fsm.cur_footstep == 1 || fsm.next_footstep == footsteps.size() - 1)
        {
            a = a / 2.f;
        }
        x = a * (theta - sin(theta));                       //< footsteps.x() += x
        z = a * (1.f - cos(theta)) / 2.f / a * STEPHEIGHT_; //< footsteps.z() += z
    }

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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "SS: Left swing first, left swing.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "SS: Left swing first, right swing.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "SS: Right swing first, right swing.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPL: " << WPL.x() << " " << WPL.y() << " " << WPL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
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

            //! LOG
            // flog << "[" << theFrameInfo->time << "]" << "SS: Right swing first, left swing.\n";
            // flog << "[" << theFrameInfo->time << "]" << "Current footstep: " << fsm.cur_footstep << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPO: " << WPO.x() << " " << WPO.y() << " " << WPO.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "LPSL: " << LPSL.x() << " " << LPSL.y() << " " << LPSL.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetL: " << targetL.translation.x() << " " << targetL.translation.y() << " " << targetL.translation.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "WPR: " << WPR.x() << " " << WPR.y() << " " << WPR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "RPSR: " << RPSR.x() << " " << RPSR.y() << " " << RPSR.z() << std::endl;
            // flog << "[" << theFrameInfo->time << "]" << "targetR: " << targetR.translation.x() << " " << targetR.translation.y() << " " << targetR.translation.z() << std::endl;
        }
    }

    //! Calculate jointRequest.
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    //! Log
    // printf("SingleSupport>\n");
    // printf("targetL: %f %f %f\n", targetL.translation.x(), targetL.translation.y(), targetL.translation.z());
    // printf("targetR: %f %f %f\n", targetR.translation.x(), targetR.translation.y(), targetR.translation.z());
    // printf("----\n\n");

    //! Update BalanceTarget
    if (isPossible)
    {
        theBalanceTarget->lastJointRequest = jointRequest_;
        theBalanceTarget->soleLeftRequest = targetL;
        theBalanceTarget->soleRightRequest = targetR;
    }
    else
    {
        printf("[WARNING]: In calcJointInSingleSupport(), foot target cannot reach.\n");
    }
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

void FootstepsController::standRebalance()
{
    //! Check balance state.
    const float gyroX = theInertialData->gyro.y();
    const float angleX = theInertialData->angle.x();

    const float MAX_ANGLE_ROTATE = 3_deg;
    const float MAX_GYRO_ROTATE = 3_deg;

    if (abs(gyroX) < MAX_GYRO_ROTATE && abs(angleX < MAX_ANGLE_ROTATE))
        standRebalanceCounter++;
    else
        standRebalanceCounter = 0;

    if (standRebalanceCounter > 5)
    {
        fsm.state = WalkingFSM::recovery;
        recoveryStartTime_ = theFrameInfo->time;
        recoveryStartJointRequest_ = jointRequest_;
        return recoveryToStand();
    }

    //! Run stand balance.
    const float MAX_GYRO_Y = 0.1f;
    const float MAX_ANGLE_X = MAX_ANGLE_ROTATE;
    const float STEP_DEPTH = 1.f;

    //! Update by cycle.
    if (standRebalanceCycle-- == 0)
    {
        standRebalanceCycle = STAND_REBALANCE_CYCLE;
        //! Normalized gyroX to (-1, 1)
        // float normalizedGyroX = gyroX / MAX_GYRO_Y;
        // if (normalizedGyroX > 1.f)
        //     normalizedGyroX = 1.f;
        // else if (normalizedGyroX < -1.f)
        //     normalizedGyroX = -1.f;

        // standRebalanceZ = normalizedGyroX * STEP_DEPTH;

        //! Normalized angleX to (-1, 1).
        float normalizedAngleX = angleX / MAX_ANGLE_ROTATE;
        if (normalizedAngleX > 1.f)
            normalizedAngleX = 1.f;
        else if (normalizedAngleX < -1.f)
            normalizedAngleX = -1.f;

        standRebalanceZ = normalizedAngleX * STEP_DEPTH;
        return;
    }

    //! Last sole left and right targets.
    Pose3f targetL = theBalanceTarget->soleLeftRequest;
    Pose3f targetR = theBalanceTarget->soleRightRequest;

    float ratio = (float)(STAND_REBALANCE_CYCLE - standRebalanceCycle) / STAND_REBALANCE_CYCLE;
    float z = standRebalanceZ * sin(pi * ratio);
    targetL.translation.z() = -(hipHeight_ + z);
    targetR.translation.z() = -(hipHeight_ - z);

    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), jointRequest_, *theRobotDimensions);
    updatedJointRequest = isPossible;

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;
}

void FootstepsController::recoveryToStand()
{
    const float T = 0.5f;
    float t = (float)(theFrameInfo->time - recoveryStartTime_) / 1000.f;
    Pose3f targetL = Pose3f(Vector3f(-30.f, theRobotDimensions->yHipOffset, -hipHeight_));
    Pose3f targetR = Pose3f(Vector3f(-30.f, -theRobotDimensions->yHipOffset, -hipHeight_));
    JointRequest j;
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), j, *theRobotDimensions);
    updatedJointRequest = isPossible;

    for (int i = 0; i <= Joints::rAnkleRoll; i++)
    {
        jointRequest_.angles[i] = recoveryStartJointRequest_.angles[i] + t / T * (j.angles[i] - recoveryStartJointRequest_.angles[i]);
    }

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
    theBalanceTarget->soleLeftRequest = targetL;
    theBalanceTarget->soleRightRequest = targetR;

    //! return to initial standing.
    if (t > T)
    {
        return startStanding();
    }
}

void FootstepsController::balance()
{
    ankleBalance();

    //! Update BalanceTarget
    theBalanceTarget->lastJointRequest = jointRequest_;
}

void FootstepsController::ankleBalance()
{
    StanceFoot stance = getStanceFoot();

    switch (stance)
    {
    case StanceFoot::left:
        leftAnkleBalance();
        break;
    case StanceFoot::right:
        rightAnkleBalance();
    default:
        break;
    }
}

void FootstepsController::leftAnkleBalance()
{
    // const Vector3a &gyro = theInertialData->gyro;
    // float gX = gyro.x();
    // const float Kx = 1.f;
    // float lAnkle = Kx * gX;
    // jointRequest_.angles[Joints::lAnkleRoll] += lAnkle;
}

void FootstepsController::rightAnkleBalance()
{
    // const Vector3a &gyro = theInertialData->gyro;
    // float gX = gyro.x();
    // const float Kx = 1.f;
    // float rAnkle = Kx * gX;
    // jointRequest_.angles[Joints::rAnkleRoll] += rAnkle;
}

FootstepsController::StanceFoot FootstepsController::getStanceFoot()
{
    StanceFoot stance = StanceFoot::left;
    bool left = swingLeftFirst;

    if (left)
    {
        if (fsm.cur_footstep % 2 == 1)
        {
            stance = StanceFoot::right;
        }
        else
        {
            stance = StanceFoot::left;
        }
    }
    else
    {
        if (fsm.cur_footstep % 2 == 1)
        {
            stance = StanceFoot::left;
        }
        else
        {
            stance = StanceFoot::right;
        }
    }

    return stance;
}

void FootstepsController::test()
{
    const float x = hipInitialPos_.x();
    const float y = hipInitialPos_.y();
    static float t = 0.f;
    float yOffset = com.y() - comInitialPos_.y();

    float a = 80.f / 2.f / pi;
    const float T = 1.f;
    float theta = t < T ? t / T : 1.f;
    float WTL_x = 0.f;
    float WTL_y = 55.62f;
    float WTL_z = 0.f;

    // yOffset = 15.8f * theta;

    //! WTO: bhuman frame in world frame.
    Vector3f WPO = {x, y + yOffset * 3.f, hipHeight_};
    // Vector3f WPO = {x, y, hipHeight_};
    Matrix3f WRO = Matrix3f::Identity();
    sva::PTransform WTO = {WRO, WPO};

    //! WTL: left contact surface frame in world frame.

    // float theta = t < T ? t / T * 2 * pi : 2 * pi;
    // float WTL_x = a * (theta - sin(theta));
    // float WTL_y = 55.62f;
    // float WTL_z = (1 - cos(theta)) * 5.f;

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
