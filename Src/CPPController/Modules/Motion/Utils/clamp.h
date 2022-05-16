#pragma once

#include <iostream>

void clampInPlace(float &v, float vMin, float vMax, std::string label)
{
    if (v > vMax)
    {
        std::cout << label << " clamped to " << vMax << std::endl;
        ;
        v = vMax;
    }
    else if (v < vMin)
    {
        std::cout << label << " clamped to " << vMin << std::endl;
        v = vMin;
    }
}
