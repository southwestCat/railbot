#pragma once

#include "Tools/Module/Blackboard.h"
#include "Tools/Module/BlackboardThread.h"
#include "Platform/Timer.h"
#include <pthread.h>
#include <string>
#include <iostream>
#include <unistd.h>

extern bool attemptingShutdown;

class ThreadManager;

struct SafelyRunArgs
{
    ThreadManager *threadManager;
    BlackboardThread *blackboard;
};

template <class T, void (T::*mem_fn)(BlackboardThread *)>
void *thunk(void *args);

class ThreadManager
{
public:
    ThreadManager(std::string name, int cycleTime);
    ~ThreadManager()
    {
        std::cout << "~ThreadManager " << name << std::endl;
        join();
    }
    template <class T>
    void run(BlackboardThread *bb);

    void join();
    bool setCPU(pthread_t &a, int i);

    std::string name; // thread name
    int cycleTime;    // microseconds
    pthread_t pthread;
    bool running;

private:
    template <class T>
    void safelyRun(BlackboardThread *bb);
    SafelyRunArgs args;
};

template <class T, void (T::*mem_fn)(BlackboardThread *)>
void *thunk(void *args)
{
    void *p = ((SafelyRunArgs *)args)->threadManager;
    BlackboardThread *b = ((SafelyRunArgs *)args)->blackboard;
    (static_cast<T *>(p)->*mem_fn)(b);
    return 0;
}

template <class T>
void ThreadManager::run(BlackboardThread *bb)
{
    args.blackboard = bb;
    args.threadManager = this;

    if (name == "Upper")
    {
        const int cpuID = 0;
        if (0 != pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T>>, &args))
        {
            std::cerr << "[ERROR] Cannot create thread: " << name << std::endl;
            exit(-1);
        }
        if (!setCPU(pthread, cpuID))
        {
            std::cerr << "[ERROR] Cannot set Logger thread to cpu" << cpuID << std::endl;
        }
        std::cout << "[INFO] Create thread: " << name << "     --> binding to cpu" << cpuID << std::endl;
    }
    else if (name == "Lower")
    {
        const int cpuID = 1;
        if (0 != pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T>>, &args))
        {
            std::cerr << "[ERROR] Cannot create thread: " << name << std::endl;
            exit(-1);
        }
        if (!setCPU(pthread, cpuID))
        {
            std::cerr << "[ERROR] Cannot set Logger thread to cpu" << cpuID << std::endl;
        }
        std::cout << "[INFO] Create thread: " << name << "     --> binding to cpu" << cpuID << std::endl;
    }
    else if (name == "Cognition")
    {
        const int cpuID = 2;
        if (0 != pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T>>, &args))
        {
            std::cerr << "[ERROR] Cannot create thread: " << name << std::endl;
            exit(-1);
        }
        if (!setCPU(pthread, cpuID))
        {
            std::cerr << "[ERROR] Cannot set Cognition thread to cpu1" << std::endl;
            exit(-1);
        }
        std::cout << "[INFO] Create thread: " << name << " --> binding to cpu" << cpuID << std::endl;
    }
    else if (name == "Motion")
    {
        const int cpuID = 3;
        if (0 != pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T>>, &args))
        {
            std::cerr << "[ERROR] Cannot create thread: " << name << std::endl;
            exit(-1);
        }
        if (!setCPU(pthread, cpuID))
        {
            std::cerr << "[ERROR] Cannot set Motion thread to cpu" << cpuID << std::endl;
            exit(-1);
        }
        std::cout << "[INFO] Create thread: " << name << "    --> binding to cpu" << cpuID << std::endl;
    }
    else
    {
        if (0 != pthread_create(&pthread, NULL, &thunk<ThreadManager, &ThreadManager::safelyRun<T>>, &args))
        {
            std::cerr << "[ERROR] Cannot create thread: " << name << std::endl;
            exit(-1);
        }
        std::cout << "[INFO] Create thread: " << name << std::endl;
    }
    running = true;
}

template <class T>
void ThreadManager::safelyRun(BlackboardThread *bb)
{
    T t(bb);
    Timer timer;
    unsigned int elapsed = 0;
    while (!attemptingShutdown)
    {
        try
        {
            timer.reset();
            if (cycleTime != -1)
            {
                t.tick();
                elapsed = timer.elapsed_us();
                if ((int)elapsed < cycleTime)
                {
                    usleep(cycleTime - elapsed);
                }
            }
            else
            {
                sleep(1);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}
