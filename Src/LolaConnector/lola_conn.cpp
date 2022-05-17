#include <cmath>
#include <csignal>
#include <cstdio>
#include <cassert>
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
uint64_t CycleIndex = 0;
uint64_t startPressTime = 0;
int lastReadingActuators = -1;
int actuatorsDrop = 0;

enum State
{
    running,
    poweroff
} state;

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
        perror("[ERROR]: shm_open.");
        return -1;
    }
    else if (ftruncate(memoryHandle, sizeof(SharedData)) == -1)
    {
        perror("[ERROR]: ftruncate.");
        return -1;
    }
    else
    {
        shData = (SharedData *)mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
        if (shData == MAP_FAILED)
        {
            perror("[ERROR]: mmap.");
            return -1;
        }
        else
        {
            memset((void *)shData, 0, sizeof(SharedData));
            sem = sem_open(SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
            if (sem == SEM_FAILED)
            {
                perror("[ERROR]: sem_open.");
                return -1;
            }
        }
    }
    shData->state = 1;
    return 0;
}

void closeSharedMemory()
{
    fprintf(stderr, "[INFO]: lola_conn is stopping.\n");
    if (sem != SEM_FAILED)
        sem_close(sem);
    if (shData != MAP_FAILED)
        munmap(shData, sizeof(SharedData));

    fprintf(stderr, "[INFO]: lola_conn is stopped.\n");
}

void extractSensors(float *sensors, const LolaSensorFrame &sensor_frame)
{
    int index = 0;
    for (int i = 0; i < HEAD_JOINT_SIZE; i++)
    {
        sensors[index++] = sensor_frame.joints.head[i].angle;
        sensors[index++] = sensor_frame.joints.head[i].current;
        sensors[index++] = sensor_frame.joints.head[i].temperature;
    }
    for (int i = 0; i < ARM_JOINT_SIZE; i++)
    {
        sensors[index++] = sensor_frame.joints.arms[i].angle;
        sensors[index++] = sensor_frame.joints.arms[i].current;
        sensors[index++] = sensor_frame.joints.arms[i].temperature;
    }
    for (int i = 0; i < LEG_JOINT_SIZE; i++)
    {
        sensors[index++] = sensor_frame.joints.legs[i].angle;
        sensors[index++] = sensor_frame.joints.legs[i].current;
        sensors[index++] = sensor_frame.joints.legs[i].temperature;
    }
    sensors[index++] = sensor_frame.touch.headTouch.front;
    sensors[index++] = sensor_frame.touch.headTouch.middle;
    sensors[index++] = sensor_frame.touch.headTouch.rear;
    sensors[index++] = sensor_frame.touch.lHandTouch.back;
    sensors[index++] = sensor_frame.touch.lHandTouch.left;
    sensors[index++] = sensor_frame.touch.lHandTouch.right;
    sensors[index++] = sensor_frame.touch.rHandTouch.back;
    sensors[index++] = sensor_frame.touch.rHandTouch.left;
    sensors[index++] = sensor_frame.touch.rHandTouch.right;
    sensors[index++] = sensor_frame.touch.lFootBumper.left;
    sensors[index++] = sensor_frame.touch.lFootBumper.right;
    sensors[index++] = sensor_frame.touch.rFootBumper.left;
    sensors[index++] = sensor_frame.touch.rFootBumper.right;
    sensors[index++] = sensor_frame.touch.ChestBtn;
    sensors[index++] = sensor_frame.imu.gyr.roll;
    sensors[index++] = sensor_frame.imu.gyr.pitch;
    sensors[index++] = sensor_frame.imu.gyr.yaw;
    sensors[index++] = sensor_frame.imu.accel.x;
    sensors[index++] = sensor_frame.imu.accel.y;
    sensors[index++] = sensor_frame.imu.accel.z;
    sensors[index++] = sensor_frame.imu.angles.X;
    sensors[index++] = sensor_frame.imu.angles.Y;
    sensors[index++] = sensor_frame.imu.angles.Z;
    sensors[index++] = sensor_frame.battery.current;
    sensors[index++] = sensor_frame.battery.charge;
    sensors[index++] = sensor_frame.battery.status;
    sensors[index++] = sensor_frame.battery.temp;
    sensors[index++] = sensor_frame.fsr.left.fl;
    sensors[index++] = sensor_frame.fsr.left.fr;
    sensors[index++] = sensor_frame.fsr.left.rl;
    sensors[index++] = sensor_frame.fsr.left.rr;
    sensors[index++] = sensor_frame.fsr.right.fl;
    sensors[index++] = sensor_frame.fsr.right.fr;
    sensors[index++] = sensor_frame.fsr.right.rl;
    sensors[index++] = sensor_frame.fsr.right.rr;
    sensors[index++] = sensor_frame.fsr.left.fl +
                       sensor_frame.fsr.left.fr +
                       sensor_frame.fsr.left.rl +
                       sensor_frame.fsr.left.rr;
    sensors[index++] = sensor_frame.fsr.right.fl +
                       sensor_frame.fsr.right.fr +
                       sensor_frame.fsr.right.rl +
                       sensor_frame.fsr.right.rr;
}

void setEyeLeds(float *request)
{
    static int i = 0;
    if (i < 10)
    {
        request[faceLedBlueLeft135DegActuator - faceLedRedLeft0DegActuator] = 1.f;
        request[faceLedBlueRight225DegActuator - faceLedRedLeft0DegActuator] = 1.f;
    }
    if (i == 20)
        i = 0;
    i++;
}

void writeLEDs(Leds &leds, float *ledRequest)
{
    // lEye Red
    int index = 0;
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.left[i].r = ledRequest[index++];
    }
    // lEye Green
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.left[i].g = ledRequest[index++];
    }
    // lEye Blue
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.left[i].b = ledRequest[index++];
    }
    // rEye Red
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.right[i].r = ledRequest[index++];
    }
    // rEye Green
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.right[i].g = ledRequest[index++];
    }
    // rEye Blue
    for (int i = 0; i < 8; i++)
    {
        leds.eyes.right[i].b = ledRequest[index++];
    }
    // lEar
    for (int i = 0; i < 10; i++)
    {
        leds.ears.left[i] = ledRequest[index++];
    }
    // rEar
    for (int i = 0; i < 10; i++)
    {
        leds.ears.right[i] = ledRequest[index++];
    }
    // chest
    leds.chestLed.r = ledRequest[index++];
    leds.chestLed.g = ledRequest[index++];
    leds.chestLed.b = ledRequest[index++];
    // head led
    for (int i = 0; i < 12; i++)
    {
        leds.head[i] = ledRequest[index++];
    }
    // foot led
    leds.foots.left.r = ledRequest[index++];
    leds.foots.left.g = ledRequest[index++];
    leds.foots.left.b = ledRequest[index++];
    leds.foots.right.r = ledRequest[index++];
    leds.foots.right.g = ledRequest[index++];
    leds.foots.right.b = ledRequest[index++];
}

void writeJoints(Joints &joints, float *positionRequest, float *stiffnessRequest)
{
    joints.head[HeadYaw].angle = positionRequest[headYawPositionActuator];
    joints.head[HeadYaw].stiffness = stiffnessRequest[headYawPositionActuator];
    joints.head[HeadPitch].angle = positionRequest[headPitchPositionActuator];
    joints.head[HeadPitch].stiffness = stiffnessRequest[headPitchPositionActuator];
    for (int i = 0; i < ARM_JOINT_SIZE; i++)
    {
        joints.arms[i].angle = positionRequest[lShoulderPitchPositionActuator + i];
        joints.arms[i].stiffness = stiffnessRequest[lShoulderPitchPositionActuator + i];
    }
    for (int i = 0; i < LEG_JOINT_SIZE; i++)
    {
        joints.legs[i].angle = positionRequest[lHipYawPitchPositionActuator + i];
        joints.legs[i].stiffness = stiffnessRequest[lHipYawPitchPositionActuator + i];
    }
}

void writeActuators(local::stream_protocol::socket &socket)
{
    LolaFrameHandler frame_handler;
    try
    {
        shData->readingActuators = shData->newestActuators;
        if (shData->readingActuators == lastReadingActuators)
        {
            if (actuatorsDrop == 0)
                fprintf(stderr, "[INFO]: missed actuator request. \n");
            actuatorsDrop++;
        }
        else
            actuatorsDrop = 0;
        lastReadingActuators = shData->readingActuators;

        float *readingActuators = shData->actuators[shData->readingActuators];
        float *actuators = readingActuators;
        auto &leds = frame_handler.actuator_frame.leds;

        float positionRequest[numOfPositionActuatorIds];
        float stiffnessRequest[numOfPositionActuatorIds];
        for (int i = 0; i < numOfPositionActuatorIds; i++)
        {
            positionRequest[i] = actuators[i];
        }
        for (int i = 0; i < numOfStiffnessActuatorIds; i++)
        {
            stiffnessRequest[i] = actuatorsDrop > 5 ? 0.f : actuators[headYawStiffnessActuator + i];
        }

        auto &joints = frame_handler.actuator_frame.joints;
        writeJoints(joints, positionRequest, stiffnessRequest);

        float ledRequest[numOfLedActuatorIds] = {0.f};
        int ledIndex = 0;

        if (actuatorsDrop > 5)
        {
            setEyeLeds(ledRequest);
            writeLEDs(leds, ledRequest);
        }
        else
        {
            for (int i = 0; i < numOfLedActuatorIds; i++)
            {
                int index = faceLedRedLeft0DegActuator + ledIndex;
                ledIndex++;
                ledRequest[i] = actuators[index];
            }
            writeLEDs(leds, ledRequest);
        }

        char *buffer;
        size_t size;
        // std::cout<<"the right wrist is "<< joints.arms[RWristYaw].angle<<std::endl;
        tie(buffer, size) = frame_handler.pack();
        socket.send(boost::asio::buffer(buffer, size));
    }
    catch (string &e)
    {
    }
}

void readSensors(local::stream_protocol::socket &socket, LolaFrameHandler &frame_handler, char *socketdata, int max_len)
{
    try
    {
        const LolaSensorFrame &sensor_frame = frame_handler.unpack(socketdata, socket.receive(boost::asio::buffer(socketdata, max_len)));
        int writingSensors = 0;
        if (writingSensors == shData->newestSensors)
            writingSensors++;
        if (writingSensors == shData->readingSensors)
            if (++writingSensors == shData->newestSensors)
                writingSensors++;
        assert(writingSensors != shData->newestSensors);
        assert(writingSensors != shData->readingSensors);
        //! read sensors and write data to shared memory.
        float *sensors = shData->sensors[writingSensors];
        extractSensors(sensors, sensor_frame);
        shData->newestSensors = writingSensors;
        //! press chest button to shotdown robot.
        if (sensors[chestButtonSensor] == 0.f)
            startPressTime = CycleIndex;
        else if (state != poweroff && startPressTime && CycleIndex - startPressTime > 100)
        {
            (void)!system("sudo systemctl poweroff &");
            state = poweroff;
        }
    }
    catch (string &e)
    {
        fprintf(stderr, "[ERROR]: catch %s\n", e.c_str());
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
        CycleIndex++;
        writeActuators(socket);
        readSensors(socket, frame_handler, socketdata, max_len);
        // raise the semaphore
        if (sem != SEM_FAILED)
        {
            int sval;
            if (sem_getvalue(sem, &sval) == 0)
            {
                if (sval < 1)
                {
                    sem_post(sem);
                }
            }
        }
    }
}

int main()
{
    registerSignal();

    int ret = openSharedMemory();
    if (ret < 0)
    {
        printf("[ERROR]: Cannot open shared memory.\n");
        return -1;
    }

    state = running;

    processReceiveSend();

    closeSharedMemory();

    return 0;
}