/**
 * @date 2019-08-18 Sun 22:18:56 CST
 * ====
 * This file implements a timer object measures elapsed time.
 * ====
 * clock_gettime() get different value in Nao and laptop. In nao robot, 
 * the time.tv_sec is alway start in zero, but in your laptop 
 * it's UTC(number is large, like 1e9). So in this class we still use 
 * unsigned int to represent the return value of msec. It's legal in 
 * nao robot, but illegal in your computer.
 * ====
 * @author Zihan (=^o^=)
 *                ,
 *              _/((
 *     _.---. .'   `\
 *   .'      `     ^ T=
 *  /     \       .--'
 * |      /       )'-.
 * ; ,   <__..-(   '-.)
 *  \ \-.__)    ``--._)
 *   '.'-.__.-.
 *
 */ 

#pragma once

#include <ctime>

class Timer
{
public:
    Timer()
    {
        clock_gettime(CLOCK_MONOTONIC, &timestamp);
    }
    void reset()
    {
        clock_gettime(CLOCK_MONOTONIC, &timestamp);
    }
    unsigned int msec(const timespec &ts)
    {
        return (unsigned int)(ts.tv_sec*1000 + ts.tv_nsec / 1000000l);
    }
    unsigned long long usec(const timespec &ts)
    {
        return ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
    }
    unsigned int getMsec()
    {
        return (unsigned int)(timestamp.tv_sec*1000 + timestamp.tv_nsec / 1000000l);
    }

    /* return elapsed time */
    unsigned int elapsed_ms()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return msec(ts) - msec(timestamp);
    }

    unsigned long long elapsed_us()
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return usec(ts) - usec(timestamp);
    }

private:
    timespec timestamp;
};