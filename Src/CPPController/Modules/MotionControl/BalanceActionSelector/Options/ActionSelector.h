define_option(ActionRoot)
{
    float comX;
    const float &xd = theFloatingBaseEstimation->comVelocity.x();
    const float ea = Selector::Ea;
    const float eb = Selector::Eb;
    const float ec = Selector::Ec;
    const float ed = Selector::Ed;
    const float eca = Selector::ECa;
    const float ecb = Selector::ECb;
    const float ecc = Selector::ECc;
    const float ecd = Selector::ECd;

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
            // printf("[INFO]: In wait.\n");
            action = BalanceActionSelection::compliance;
        }
    }

    define_state(compliance)
    {
        define_transition
        {
            // if (comX > ec || comX < eb)
            // {
            //     printf("[SELECTOR] comX: %f\n", comX);
            //     goto footstep;
            // }
            // if (xd < ecb || xd > ecc)
            // {
            //     printf("[SELECTOR] xd: %f\n", xd);
            //     goto footstep;
            // }
                
            if (theBalanceTarget->isComplianceControlDone)
                goto dcm;
        }
        define_action
        {
            printf(">\n");
            printf("[SELECTOR] In compliance.\n");
            printf("[SELECTOR] comX: %3.3f %d\n", comX, theBalanceTarget->isComplianceControlDone);
            printf("----\n\n");
            action = BalanceActionSelection::compliance;
        }
    }

    define_state(dcm)
    {
        define_transition
        {
            // if (theBalanceTarget->isDCMControlDone)
            //     goto compliance;
        }
        define_action
        {
            printf(">\n");
            printf("[SELECTOR] In DCM.\n");
            printf("----\n\n");
            action = BalanceActionSelection::dcm;
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
            // printf("[INFO]: In footstep.\n");
        }
    }
}