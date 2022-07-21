#include "FuzzyPID.h"
#include <iostream>
#include <cmath>
#include <cassert>

using std::cout;
using std::endl;

float FuzzyPID::getU(float x, float xd)
{
    //! assert not configuration
    assert(calculated);
    assert(eMin != MinNoSet);
    assert(eMax != MaxNoSet);
    assert(ecMin != MinNoSet);
    assert(ecMax != MaxNoSet);

    //! Get fuzzification inputs
    int e = fuzzificationE(x);
    int ec = fuzzificationEC(xd);

    //! assert e and ec out of range.
    assert(e <= 6);
    assert(e >= -6);
    assert(ec <= 6);
    assert(ec >= -6);

    float r = Fuzzy_Table[e + 6][ec + 6];
    if (r > 0)
    {
        if (r < 40)
            r = 40;
    }
    else
    {
        if (r > -40)
            r = -40;
    }
    return r;
}

void FuzzyPID::printFuzzyTable()
{
    cout << "Fuzzy Table: \n";
    for (int i = 0; i < 13; i++)
    {
        for (int j = 0; j < 13; j++)
        {
            cout << Fuzzy_Table[i][j] << "\t";
        }
        cout << endl;
    }
}

int FuzzyPID::E_MAX(int e)
{
    int max = 0;
    for (int i = 0; i < 7; i++)
    {
        if (E_Membership[i][e] > E_Membership[max][e])
            max = i;
    }
    return max;
}

int FuzzyPID::EC_MAX(int ec)
{
    int max = 0;
    for (int i = 0; i < 7; i++)
    {
        if (EC_Membership[i][ec] > EC_Membership[max][ec])
            max = i;
    }
    return max;
}

void FuzzyPID::updateSTEPS(const float step)
{
    const float maxStep = step;
    const float minStep = 40.f;

    assert(maxStep < minStep);

    float n = (maxStep - minStep) / 5.f;
    DynamicSTEPS[0] = -maxStep;
    DynamicSTEPS[1] = -maxStep + n;
    DynamicSTEPS[2] = -maxStep + 2.f * n;
    DynamicSTEPS[3] = -maxStep + 3.f * n;
    DynamicSTEPS[4] = -maxStep + 4.f * n;
    DynamicSTEPS[5] = -minStep;
    DynamicSTEPS[6] = 0.f;
    DynamicSTEPS[7] = minStep;
    DynamicSTEPS[8] = maxStep - 4.f * n;
    DynamicSTEPS[9] = maxStep - 3.f * n;
    DynamicSTEPS[10] = maxStep - 2.f * n;
    DynamicSTEPS[11] = maxStep - n;
    DynamicSTEPS[12] = maxStep;
}

void FuzzyPID::calcFuzzyTable()
{
    float R[169][13] = {0};

    float R1[13][13] = {0};
    float R2[169] = {0};
    float R3[169][13] = {0};

    float R1d[13][13];
    float R2d[169];

    for (int E_Index = 0; E_Index < 7; E_Index++)
    {
        for (int EC_Index = 0; EC_Index < 7; EC_Index++)
        {
            int k = 0;
            int Output_Index = Rules[E_Index][EC_Index] - 1;
            for (int i = 0; i < 13; i++)
            {
                for (int j = 0; j < 13; j++)
                {
                    if (E_Membership[E_Index][i] < EC_Membership[EC_Index][j])
                        R1[i][j] = E_Membership[E_Index][i];
                    else
                        R1[i][j] = EC_Membership[EC_Index][j];
                    R2[k] = R1[i][j];
                    k++;
                }
            }

            for (int i = 0; i < 169; i++)
            {
                for (int j = 0; j < 13; j++)
                {
                    if (R2[i] < Output_Membership[Output_Index][j])
                        R3[i][j] = R2[i];
                    else
                        R3[i][j] = Output_Membership[Output_Index][j];

                    if (R3[i][j] > R[i][j])
                        R[i][j] = R3[i][j];
                }
            }
        }
    }

    for (int E_Index = 0; E_Index < 13; E_Index++)
    {
        for (int EC_Index = 0; EC_Index < 13; EC_Index++)
        {
            float Cd[13] = {0};
            //! reset Cd[]
            for (int i = 0; i < 13; i++)
                Cd[i] = 0.f;
            int kd = 0;
            int Ed = E_MAX(E_Index);
            int ECd = EC_MAX(EC_Index);
            for (int i = 0; i < 13; i++)
            {
                for (int j = 0; j < 13; j++)
                {
                    if (E_Membership[Ed][i] < EC_Membership[ECd][j])
                        R1d[i][j] = E_Membership[Ed][i];
                    else
                        R1d[i][j] = EC_Membership[ECd][j];

                    R2d[kd] = R1d[i][j];
                    kd++;
                }
            }

            float tmp;
            for (int i = 0; i < 169; i++)
            {
                for (int j = 0; j < 13; j++)
                {
                    if (R2d[i] < R[i][j])
                        tmp = R2d[i];
                    else
                        tmp = R[i][j];

                    if (tmp > Cd[j])
                        Cd[j] = tmp;
                }
            }

            float sum1 = 0.f;
            float sum2 = 0.f;
            float out;
            for (int i = 0; i < 13; i++)
            {
                sum1 += Cd[i];
                sum2 += Cd[i] * STEPS[i];
            }
            out = (int)(sum2 / sum1 + 0.5f);
            Fuzzy_Table[E_Index][EC_Index] = out;
        }
    }

    calculated = true;
}

void FuzzyPID::calculate()
{
    if (calculated)
        return;

    return calcFuzzyTable();
}

void FuzzyPID::recalculate()
{
    return calcFuzzyTable();
}

int FuzzyPID::fuzzificationE(float e)
{
    // const float a = eMin;
    // const float b = eMax;
    // int f = round((e - (a + b) / 2.f) * 2.f * 6.f / (b - a));

    // if (f > 6)
    //     f = 6;
    // if (f < -6)
    //     f = -6;
    // return f;

    assert(e_a != MinNoSet);
    assert(e_b != MinNoSet);
    assert(e_c != MaxNoSet);
    assert(e_d != MaxNoSet);

    const float a = e_a;
    const float b = e_b;
    const float c = e_c;
    const float d = e_d;

    int f;
    if (e > 0)
    {
        f = round((e - c) / (d - c) * 6.f);
    }
    else
    {
        f = round((b - e) / (a - b) * 6.f);
    }

    if (f > 6)
        f = 6;
    if (f < -6)
        f = -6;
    return f;
}

int FuzzyPID::fuzzificationEC(float ec)
{
    // const float a = ecMin;
    // const float b = ecMax;
    // int f = round((ec - (a + b) / 2.f) * 2.f * 6.f / (b - a));

    // if (f > 6)
    //     f = 6;
    // if (f < -6)
    //     f = -6;
    // return f;

    assert(ec_a != MinNoSet);
    assert(ec_b != MinNoSet);
    assert(ec_c != MaxNoSet);
    assert(ec_d != MaxNoSet);

    const float a = ec_a;
    const float b = ec_b;
    const float c = ec_c;
    const float d = ec_d;

    int f;
    if (ec > 0)
    {
        f = round((ec - c) / (d - c) * 6.f);
    }
    else
    {
        f = round((b - ec) / (a - b) * 6.f);
    }

    if (f > 6)
        f = 6;
    if (f < -6)
        f = -6;
    return f;
}
