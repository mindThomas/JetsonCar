#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>

#include "realsense/realsense.h"

void PoseCallback(Eigen::Affine3f pose);

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);
    std::cout << "Realsense Test" << std::endl;

    Realsense realsense;

    realsense.Connect();

    realsense.RegisterCallback_Pose(PoseCallback);

    while (!ShouldExit)
    {
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    realsense.Disconnect();
}

void PoseCallback(Eigen::Affine3f pose)
{
    auto XYZ = Realsense::GetPoseTranslation(pose);
    auto RPY = Realsense::GetPoseEulerRPY(pose, true);
    printf("XYZ = %2.2f, %2.2f, %2.2f\n", XYZ(0), XYZ(1), XYZ(2));
    printf("RPY = %2.2f, %2.2f, %2.2f\n\n", RPY(0), RPY(1), RPY(2));
}