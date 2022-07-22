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
    /*
     * a < b < 0 < c < d;
     */
    void setERange(float a, float b, float c, float d)
    {
        e_a = a;
        e_b = b;
        e_c = c;
        e_d = d;
    }
    void setECRange(float a, float b, float c, float d)
    {
        ec_a = a;
        ec_b = b;
        ec_c = c;
        ec_d = d;
    }
    void updateSTEPS(const float step);
    void setMaxStep(const float s)
    {
        maxStep = s;
    }
    float getU(float x, float xd);
    float getUDynamic(float x, float xd);
    void printFuzzyTable();
    void printFuzzyTableDynamic();
    void calculate();
    void calculateDynamic();
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

    float e_a = MinNoSet;
    float e_b = MinNoSet;
    float e_c = MaxNoSet;
    float e_d = MaxNoSet;
    float ec_a = MinNoSet;
    float ec_b = MinNoSet;
    float ec_c = MaxNoSet;
    float ec_d = MaxNoSet;

    const float step = 13.f;
    const float STEPS[13] = {-90, -80, -70, -60, -50, -40, 0, 40, 50, 60, 70, 80, 90};
    float DynamicSTEPS[13] = {0.f};
    const float E_Membership[7][13] =
        {1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.5, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 1, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.5, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 0.5, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1};

    const float EC_Membership[7][13] =
        {1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.5, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 1, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.5, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 0.5, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1};

    const float Output_Membership[7][13] =
        {1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.5, 1, 0.2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 1, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.2, 1, 0.5, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 0.5, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1};

    // const int Rules[7][7] =
    //     {
    //         1, 1, 1, 1, 2, 4, 4,
    //         1, 1, 1, 1, 2, 4, 4,
    //         2, 2, 2, 2, 4, 5, 5,
    //         2, 2, 3, 4, 5, 6, 6,
    //         3, 3, 4, 6, 6, 6, 6,
    //         4, 4, 6, 7, 7, 7, 7,
    //         4, 4, 6, 7, 7, 7, 7};
    const int Rules[7][7] =
        {
            1, 1, 2, 2, 3, 5, 5,
            1, 1, 2, 2, 3, 5, 5,
            1, 2, 3, 3, 3, 5, 6,
            1, 2, 3, 4, 5, 6, 7,
            2, 3, 5, 5, 5, 6, 7,
            3, 3, 5, 6, 6, 7, 7,
            3, 3, 5, 6, 6, 7, 7};

    float Fuzzy_Table[13][13] = {0};
    float Fuzzy_Table_Dynamic[13][13] = {0};

private:
    int E_MAX(int e);
    int EC_MAX(int ec);
    void calcFuzzyTable();
    float maxStep;

    bool calculated = false;
};