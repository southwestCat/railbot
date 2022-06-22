#include <iostream>
#include <csignal>
#include <unistd.h>

#include "Platform/NaoBody.h"
#include "Platform/ThreadManager.h"
#include "Processes/Motion.h"
#include "Processes/Cognition.h"
#include "Tools/Module/BlackboardThread.h"

using std::cout;
using std::endl;

static bool run = true;
bool attemptingShutdown = false;

static void sighandlerShutdown(int sig)
{
    if (run)
    {
        printf("Caught signal %i\nShutting down...\n", sig);
    }
    run = false;
    attemptingShutdown = true;
}

void registerSighandler()
{
    signal(SIGINT, sighandlerShutdown);
    signal(SIGTERM, sighandlerShutdown);
}

int main(int argc, char **argv)
{
    registerSighandler();

    NaoBody naoBody;
    if (!naoBody.init())
    {
        cout << "[INFO] Waiting for lola_conn..." << endl;
        do
        {
            usleep(1000000);
            cout << "[INFO] waiting" << endl;
            if (!run)
                exit(EXIT_SUCCESS);
        } while (!naoBody.init());
    }
    cout << "[INFO] Connected to lola_conn" << endl;

    BlackboardThread *blackboard = new BlackboardThread;

    ThreadManager motion("Motion", 0);
    motion.run<Motion>(blackboard);

    usleep(100000);

    ThreadManager behavior("Cognition", 20000);
    behavior.run<Cognition>(blackboard);

    while (run)
    {
        pause();
    }

    delete blackboard;

    exit(EXIT_SUCCESS);
}