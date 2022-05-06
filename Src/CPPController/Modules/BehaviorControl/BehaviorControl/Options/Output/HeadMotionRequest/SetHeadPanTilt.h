define_option(SetHeadPanTilt, (float) pan, (float) tilt, (float)(pi) speed)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (state_time > 200 && !theHeadMotionEngineOutput->moving)
            {
                goto targetReached;
            }
        }
        define_action
        {
            theHeadMotionRequest->pan = pan;
            theHeadMotionRequest->tilt = tilt;
            theHeadMotionRequest->speed = speed;
        }
    }

    target_state(targetReached)
    {
        define_transition
        {
            goto setRequest;
        }
        define_action
        {
            theHeadMotionRequest->pan = pan;
            theHeadMotionRequest->tilt = tilt;
            theHeadMotionRequest->speed = speed;
        }
    }
}