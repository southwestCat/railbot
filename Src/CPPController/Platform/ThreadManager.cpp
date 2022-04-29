#include "ThreadManager.h"
#include "Tools/Module/Blackboard.h"
#include <string>
#include <csignal>
#include <unistd.h>
#include <iostream>

ThreadManager::ThreadManager(std::string name, int cycleTime)
{
    this->name = name;
    this->cycleTime = cycleTime;
    running = false;
}

void ThreadManager::join()
{
    if (running)
    {
        pthread_join(pthread, NULL);
    }
    running = false;
}

bool ThreadManager::setCPU(pthread_t &a, int i)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(i, &mask);
    if (0 != pthread_setaffinity_np(a, sizeof(mask), &mask))
    {
        std::cerr << "[ERROR] Cannot set CPU affinity, aborting..." << std::endl;
        return false;
    }
    return true;
}