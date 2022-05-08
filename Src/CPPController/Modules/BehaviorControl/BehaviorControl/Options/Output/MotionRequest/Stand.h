define_option(Stand)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (theMotionInfo->motion == MotionInfo::stand)
                goto requestIsExecuted;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::stand;
        }
    }

    target_state(requestIsExecuted)
    {
        define_transition
        {
            if (theMotionInfo->motion != MotionRequest::stand)
                goto setRequest;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::stand;
        }
    }
}