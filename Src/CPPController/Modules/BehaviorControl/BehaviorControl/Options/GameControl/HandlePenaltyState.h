define_option(HandlePenaltyState)
{
    initial_state(penalized)
    {
        define_transition
        {
            if (action_done)
                goto notPenalized;
        }
        define_action
        {
            theRobotInfo->penalty = PENALTY_SPL_REQUEST_FOR_PICKUP;
            Stand();
            ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
        }
    }

    define_state(notPenalized)
    {
        define_transition
        {
            if (action_done)
                goto penalized;
        }
        define_action
        {
            theRobotInfo->penalty = PENALTY_NONE;
            HandleGameState();
        }
    }
}
