#pragma once

#include "Tools/Math/Eigen.h"

template <class T>
class RepresentationTemplate
{
public:
    RepresentationTemplate()
    {
        writting = 0;
        reading = 0;
        newest = 0;
    }
    T data[3];
    volatile int writting;
    volatile int reading;
    volatile int newest;

    void write(const T &t)
    {
        writting = 0;
        if (writting == newest)
            writting++;
        if (writting == reading)
            if (++writting == newest)
                ++writting;
        data[writting] = t;
        newest = writting;
    }

    T read()
    {
        reading = newest;
        return data[reading];
    }
};

class BlackboardThread
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    BlackboardThread();
    ~BlackboardThread();

    static BlackboardThread &getInstance();
    static void setInstance(BlackboardThread *instance);

    void *theKeyStatesThread = nullptr;
    void *theHeadMotionRequest = nullptr;
    void *theHeadMotionEngineOutput = nullptr;
};