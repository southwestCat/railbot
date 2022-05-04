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
        define_action
        {
        }
    }
}