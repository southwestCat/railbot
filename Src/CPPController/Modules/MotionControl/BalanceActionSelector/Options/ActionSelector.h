define_option(ActionRoot)
{
    float comX;
    const float &xd = theFloatingBaseEstimation->comVelocity.x();

    common_transition
    {
        Vector3f comPosition = theFloatingBaseEstimation->WTB.translation();
        comPosition.x() -= theRobotDimensions->leftAnkleToSoleCenter.x();
        comX = comPosition.x();
    }

    initial_state(wait)
    {
        define_transition
        {
            if (theBalanceTarget->balanceEngineReadyPosture)
                goto compliance;
        }
        define_action
        {
            printf("[INFO]: In wait.\n");
            action = BalanceActionSelection::compliance;
        }
    }

    define_state(compliance)
    {
        define_transition
        {
            if (abs(comX) > 20)
                goto footstep;
        }
        define_action
        {
            // printf("[INFO]: In ActionRoot. \n");
            printf("[INFO] comX: %f.\n", comX);
            action = BalanceActionSelection::compliance;
        }
    }

    define_state(dcm)
    {
        define_transition
        {
            if (theBalanceTarget->isDCMControlDone)
                goto compliance;
        }
        define_action
        {
            // action = BalanceActionSelection::dcm;
            printf("[INFO]: In DCM.\n");
        }
    }

    define_state(footstep)
    {
        define_transition
        {
            // if (state_time > 1000)
            //     goto dcm;
        }
        define_action
        {
            action = BalanceActionSelection::footstep;
            printf("[INFO]: In footstep.\n");
        }
    }
}