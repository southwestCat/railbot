define_option(ButtonPressedAndReleased, (KeyStates::Key)key, (int)releaseTimeOut, (int)successiveTimeOut)
{
    initial_state(waitingForPressInit)
    {
        define_transition
        {
            if (theKeyStates->pressed[key])
                goto waitingForRelease;
        }
    }

    define_state(waitingForPressSuccessive)
    {
        define_transition
        {
            if (state_time > successiveTimeOut)
                goto timeOut;
            if (theKeyStates->pressed[key])
                goto waitingForRelease;
        }
    }

    define_state(waitingForRelease)
    {
        define_transition
        {
            if (state_time > releaseTimeOut)
                goto timeOut;
            else if (!theKeyStates->pressed[key])
                goto success;
        }
    }

    target_state(success)
    {
        define_transition
        {
            // goto waitingForPressSuccessive;
        }
        define_action
        {
            printf("Success press.\n");
        }
    }

    aborted_state(timeOut)
    {
        define_transition
        {
            goto waitingForPressInit;
        }
    }
}