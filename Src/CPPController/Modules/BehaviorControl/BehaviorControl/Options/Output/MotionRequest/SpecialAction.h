define_option(SpecialAction, (SpecialActionRequest::SpecialActionID)id, (bool)(false)mirror)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (theMotionInfo->motion == MotionRequest::specialAction && theMotionInfo->specialActionRequest.specialAction == id)
            {
                goto requestIsExecuted;
            }
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::specialAction;
            theMotionRequest->specialActionRequest.specialAction = id;
        }
    }

    target_state(requestIsExecuted)
    {
        define_transition
        {
            if (theMotionInfo->motion != MotionRequest::specialAction || theMotionInfo->specialActionRequest.specialAction != id)
            {
                goto setRequest;
            }
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::specialAction;
            theMotionRequest->specialActionRequest.specialAction = id;
        }
    }
}