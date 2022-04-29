#pragma once

class NaoBody
{
private:
    int writingActuators = -1;
public:
    bool init();
    bool cleanup();
    bool wait();
    float* getSensors();
    void openActuators(float* &actuators);
    void closeActuators();
};