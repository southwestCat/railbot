define_option(PlayingState)
{
    initial_state(demo)
    {
        define_action
        {
            printf("In PlayingState.\n");
        }
    }
}