define_option(SpecialAction, (SpecialActionRequest::SpecialActionID)id, (bool)(false)mirror)
{
    initial_state(setRequest)
    {
        define_transition
        {
            if (theMotionInfo->motion == MotionRequest::specialAction && theMotionInfo->specialActionRequest.specialAction == id && theMotionInfo->specialActionRequest.mirror == mirror)
            {
                goto requestIsExecuted;
            }
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::specialAction;
            theMotionRequest->specialActionRequest.specialAction = id;
            theMotionRequest->specialActionRequest.mirror = mirror;
        }
    }

    target_state(requestIsExecuted)
    {
        define_transition
        {
            if (theMotionInfo->motion != MotionRequest::specialAction || theMotionInfo->specialActionRequest.specialAction != id || theMotionInfo->specialActionRequest.mirror != mirror)
            {
                goto setRequest;
            }
        }
        define_action
        {
            theMotionRequest->motion = MotionRequest::specialAction;
            theMotionRequest->specialActionRequest.specialAction = id;
            theMotionRequest->specialActionRequest.mirror = mirror;
        }
    }
}