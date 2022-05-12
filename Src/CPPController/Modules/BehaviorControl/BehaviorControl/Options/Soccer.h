define_option(Root)
{
    initial_state(playDead)
    {
        define_transition
        {
            if (action_done)
                goto standUp;
        }
        define_action
        {
            // printf("playDead.\n");
            ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
        }
    }

    define_state(standUp)
    {
        define_transition
        {
            if (action_done)
            {
                goto playSoccer;
            }
        }
        define_action
        {
            // printf("standUp.\n");
            LookForward();
            Stand();
        }
    }

    define_state(playSoccer)
    {
        define_transition
        {
            if (action_done)
                goto waitForSecondButtonPress;
        }
        define_action
        {
            // printf("playSoccer.\n");
            HandlePenaltyState();
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    define_state(waitForSecondButtonPress)
    {
        define_transition
        {
            if (action_done)
                goto waitForThirdButtonPress;
            else if (action_aborted)
                goto playSoccer;
        }
        define_action
        {
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    define_state(waitForThirdButtonPress)
    {
        define_transition
        {
            if (action_done)
                goto sitDown;
            else if (action_aborted)
                goto playSoccer;
        }
        define_action
        {
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    define_state(sitDown)
    {
        define_transition
        {
            if (action_done)
                goto playDead;
        }
        define_action
        {
            SitDown();
        }
    }
}