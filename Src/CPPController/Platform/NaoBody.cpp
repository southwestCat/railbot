#include "NaoBody.h"
#include "SharedMemoryData.h"

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cassert>
#include <iostream>

class NaoBodyAccess
{
public:
    int fd = -1;
    sem_t *sem = SEM_FAILED;
    static SharedData *shData;

    ~NaoBodyAccess()
    {
        cleanup();
    }

    bool init()
    {
        if (shData != MAP_FAILED)
            return true;

        fd = shm_open(MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
        if (fd == -1)
            return false;
        sem = sem_open(SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
        if (sem == SEM_FAILED)
        {
            close(fd);
            fd = -1;
            return false;
        }
        assert((shData = (SharedData *)mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) != MAP_FAILED);
        return true;
    }

    void cleanup()
    {
        if (shData != MAP_FAILED)
        {
            munmap(shData, sizeof(SharedData));
            shData = (SharedData *)MAP_FAILED;
        }
        if (fd != -1)
        {
            close(fd);
            fd = -1;
        }
        if (sem != SEM_FAILED)
        {
            sem_close(sem);
            sem = SEM_FAILED;
        }
    }
} naoBodyAccess;

SharedData *NaoBodyAccess::shData = (SharedData *)MAP_FAILED;

bool NaoBody::init()
{
    return naoBodyAccess.init();
}

bool NaoBody::cleanup()
{
    naoBodyAccess.cleanup();
}

bool NaoBody::wait()
{
    assert(naoBodyAccess.shData != (SharedData *)MAP_FAILED);
    assert(naoBodyAccess.sem != SEM_FAILED);
    do
    {
        if (sem_wait(naoBodyAccess.sem) == -1)
        {
            bool success = false;
            while (errno == 516)
            {
                if (sem_wait(naoBodyAccess.sem) == -1)
                {
                    assert(false);
                    continue;
                }
                else
                {
                    success = true;
                    break;
                }
            }
            if (!success)
            {
                assert(false);
                return false;
            }
        }
    } while (naoBodyAccess.shData->readingSensors == naoBodyAccess.shData->newestSensors);
    naoBodyAccess.shData->readingSensors = naoBodyAccess.shData->newestSensors;

    static bool shout = true;
    if (shout)
    {
        shout = false;
        printf("[INFO] lola_conn is working\n");
    }

    return true;
}

void NaoBody::openActuators(float *&actuators)
{
    assert(naoBodyAccess.shData != (SharedData *)MAP_FAILED);
    assert(writingActuators == -1);
    writingActuators = 0;
    if (writingActuators == naoBodyAccess.shData->newestActuators)
        ++writingActuators;
    if (writingActuators == naoBodyAccess.shData->readingActuators)
        if (++writingActuators == naoBodyAccess.shData->newestActuators)
            ++writingActuators;
    assert(writingActuators != naoBodyAccess.shData->newestActuators);
    assert(writingActuators != naoBodyAccess.shData->readingActuators);
    actuators = naoBodyAccess.shData->actuators[writingActuators];
}

void NaoBody::closeActuators()
{
    assert(naoBodyAccess.shData != (SharedData *)MAP_FAILED);
    assert(writingActuators >= 0);
    naoBodyAccess.shData->newestActuators = writingActuators;
    writingActuators = -1;
}

float *NaoBody::getSensors()
{
    assert(naoBodyAccess.shData != (SharedData*)MAP_FAILED);
    return naoBodyAccess.shData->sensors[naoBodyAccess.shData->readingSensors];
}
