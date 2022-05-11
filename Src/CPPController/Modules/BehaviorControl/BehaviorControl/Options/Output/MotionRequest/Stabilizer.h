define_option(Stabilizer)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (theMotionInfo->motion == MotionInfo::balance)
                goto requestIsExecuted;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::balance;
        }
    }

    target_state(requestIsExecuted)
    {
        define_transition
        {
            if (theMotionInfo->motion != MotionRequest::balance)
                goto setRequest;
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::balance;
        }
    }
}