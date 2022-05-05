define_option(LookForward, (float)(0.f) tilt, (float)(0.f) pan)
{
    initial_state(lookForward)
    {
        define_action
        {
            SetHeadPanTilt(pan, tilt, 10_deg);
        }
    }
}