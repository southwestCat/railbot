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
            LookForward();
            Stand();
        }
    }

    define_state(playSoccer)
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