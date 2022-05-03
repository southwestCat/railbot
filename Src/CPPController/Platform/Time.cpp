#include "Time.h"
#include <ctime>
#include <cassert>
#include <pthread.h>

unsigned Time::base = 0;
unsigned long long Time::threadTimebase = 0;

unsigned Time::getCurrentSystemTime()
{
// #ifdef TARGET_SIM
//     if (RoboCupCtrl::controller)
//         return RoboCupCtrl::controller->getTime();
// #endif
    return getRealSystemTime();
}

unsigned Time::getRealSystemTime()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // get current value of CLOCK_MONOTONIC, and store it in ts 
    const unsigned int time = (unsigned int)(ts.tv_sec*1000 + ts.tv_nsec / 1000000l);
    if (!base)
    {
        base = time - 100000; // avoid time == 0, because it is often used as a marker
    }
    return time - base;
}

unsigned long long Time::getCurrentThreadTime()
{
    clockid_t cid;
    timespec ts;

    assert(pthread_getcpuclockid(pthread_self(), &cid) == 0);
    assert(clock_gettime(cid, &ts) == 0);

    const unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
    if (!threadTimebase)
    {
        threadTimebase = time - 1000000ll;
    }
    return time - threadTimebase;
}