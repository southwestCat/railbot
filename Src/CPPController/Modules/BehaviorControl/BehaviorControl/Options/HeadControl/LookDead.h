define_option(LookDead, (float)(JointAngles::off) tilt, (float)(JointAngles::off) pan)
{
    initial_state(lookForward)
    {
        define_action
        {
            SetHeadPanTilt(pan, tilt, 10_deg);
        }
    }
}