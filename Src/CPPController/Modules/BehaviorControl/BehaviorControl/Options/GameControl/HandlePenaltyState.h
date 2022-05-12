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
            // printf("penalized.\n");
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
            // printf("not penalized.\n");
            HandleGameState();
        }
    }
}
