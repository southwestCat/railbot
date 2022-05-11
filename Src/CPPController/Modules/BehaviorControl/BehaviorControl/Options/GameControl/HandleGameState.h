define_option(HandleGameState)
{
    initial_state(ready)
    {
        define_transition
        {
            if (state_time > 1000.f)
                goto playing;
        }
        define_action
        {
            printf("ready.\n");
            Stand();
        }
    }

    define_state(playing)
    {
        define_action
        {
            printf("playing.\n");
            PlayingState();
        }
    }
}