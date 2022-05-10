define_option(SitDown)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (theMotionInfo->motion == MotionInfo::sitDown)
                goto requestIsExecuted;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::sitDown;
        }
    }

    target_state(requestIsExecuted)
    {
        define_transition
        {
            if (theMotionInfo->motion != MotionRequest::sitDown)
                goto setRequest;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::sitDown;
        }
    }
}