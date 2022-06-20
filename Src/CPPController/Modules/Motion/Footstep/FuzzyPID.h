#pragma once

class FuzzyPID
{
public:
    void setERange(float a, float b)
    {
        eMin = a;
        eMax = b;
    }
    void setECRange(float a, float b)
    {
        ecMin = a;
        ecMax = b;
    }
    float getU(float x, float xd);
    void printFuzzyTable();
    void calculate();
    void recalculate();
    int fuzzificationE(float e);
    int fuzzificationEC(float ec);

private:
    enum
    {
        MinNoSet = 10000,
        MaxNoSet = -10000
    };
    float eMin = MinNoSet;
    float eMax = MaxNoSet;
    float ecMin = MinNoSet;
    float ecMax = MaxNoSet;

    const float step = 13.f;
    const float STEPS[13] = {-6.f * step, -5.f * step, -4.f * step, -3.f * step, -2.f * step, -1.f * step, 0.f, step * 1.f, step * 2.f, step * 3.f, step * 4.f, step * 5.f, step * 6.f};
    const float E_Membership[7][13] =
        {1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1};

    const float EC_Membership[7][13] =
        {1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1};

    const float Output_Membership[7][13] =
        {1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.2, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 1};

    const int Rules[7][7] =
        {
            1, 1, 1, 1, 2, 4, 4,
            1, 1, 1, 1, 2, 4, 4,
            2, 2, 2, 2, 4, 5, 5,
            2, 2, 3, 4, 5, 6, 6,
            3, 3, 4, 6, 6, 6, 6,
            4, 4, 6, 7, 7, 7, 7,
            4, 4, 6, 7, 7, 7, 7};

    float Fuzzy_Table[13][13] = {0};

private:
    int E_MAX(int e);
    int EC_MAX(int ec);
    void calcFuzzyTable();

    bool calculated = false;
};