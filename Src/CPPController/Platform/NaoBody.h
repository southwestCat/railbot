#pragma once

#include <cstdio>

class NaoBody
{
private:
    int writingActuators = -1;

    FILE* fdCpuTemp = nullptr;
public:
    bool init();
    void cleanup();
    bool wait();
    float* getSensors();
    void openActuators(float* &actuators);
    void closeActuators();
    float getCPUTemperature();
};