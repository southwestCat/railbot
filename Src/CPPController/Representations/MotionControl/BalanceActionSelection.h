#pragma once

class BalanceActionSelection
{
public:
    enum BalanceAction
    {
        compliance,
        dcm,
        footstep,
        numOfBalanceAction,
    };
    BalanceAction targetAction = compliance;
};