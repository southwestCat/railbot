#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <ctime>
#include <map>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <semaphore.h>

#include <boost/asio.hpp>

#include "msgpack/include/msgpack.hpp"
#include "joints.h"
#include "lola_frame.h"
#include "SharedMemoryData.h"

using namespace boost::asio;
using namespace std;

static bool lola_shutdown = false;
SharedData *shData = (SharedData *)MAP_FAILED;
sem_t *sem = SEM_FAILED;

void ctrlc_handler(int)
{
    lola_shutdown = true;
}

void registerSignal()
{
    signal(SIGINT, ctrlc_handler);
    signal(SIGTERM, ctrlc_handler);
}

int openSharedMemory()
{
    int memoryHandle = shm_open(MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (memoryHandle == -1)
    {
        perror("lola_conn: shm_open.");
        return -1;
    }
    else if (ftruncate(memoryHandle, sizeof(SharedData)) == -1)
    {
        perror("lola_conn: ftruncate.");
        return -1;
    }
    else
    {
        shData = (SharedData *)mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
        if (shData == MAP_FAILED)
        {
            perror("lola_conn: mmap.");
            return -1;
        }
        else
        {
            memset(shData, 0, sizeof(SharedData));
            sem = sem_open(SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
            if (sem == SEM_FAILED)
            {
                perror("lola_conn: sem_open.");
                return -1;
            }
        }
    }
    return 0;
}

void closeSharedMemory()
{
    fprintf(stderr, "lola_conn: Stopping.\n");
    if (sem != SEM_FAILED)
        sem_close(sem);
    if (shData != MAP_FAILED)
        munmap(shData, sizeof(SharedData));

    fprintf(stderr, "lola_conn: Stopped.\n");
}

void writeActuators()
{
    try
    {
    }
    catch (string &e)
    {
    }
}

void readSensors(local::stream_protocol::socket& socket, LolaFrameHandler& frame_handler, char* socketdata, int max_len)
{
    try
    {
        const LolaSensorFrame &sensor_frame = frame_handler.unpack(socketdata, socket.receive(boost::asio::buffer(socketdata, max_len)));
        cout << sensor_frame.joints.head[HeadYaw].angle / M_PI * 180.0 << endl;
    }
    catch (string &e)
    {
        fprintf(stderr, "libbhuman: %s\n", e.c_str());
        exit(0);
    }
}

void processReceiveSend()
{
    io_service io_service;
    local::stream_protocol::socket socket(io_service);
    socket.connect("/tmp/robocup");
    constexpr int max_len = 100000;
    char socketdata[max_len] = {'\0'};
    boost::system::error_code ec;
    LolaFrameHandler frame_handler;

    const LolaSensorFrame &sensor_frame = frame_handler.unpack(socketdata, socket.receive(boost::asio::buffer(socketdata, max_len)));
    std::string headVersion = sensor_frame.robotConfig.HeadVersion;
    std::string bodyVersion = sensor_frame.robotConfig.BodyVersion;
    std::cout << "the bodyId is" << sensor_frame.robotConfig.BodyId << std::endl;
    std::cout << "the bodyversion is" << sensor_frame.robotConfig.BodyVersion << std::endl;
    std::cout << "the Headid is" << sensor_frame.robotConfig.HeadId << std::endl;
    std::cout << "the HeadVersion is" << sensor_frame.robotConfig.HeadVersion << std::endl;

    while (true)
    {
        if (lola_shutdown)
        {
            break;
        }
        writeActuators();
        readSensors(socket, frame_handler, socketdata, max_len);
    }
}

int main()
{
    registerSignal();

    int ret = openSharedMemory();
    if (ret < 0)
    {
        printf("Cannot open shared memory.\n");
        return -1;
    }

    processReceiveSend();

    closeSharedMemory();
}