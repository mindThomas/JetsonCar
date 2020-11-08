#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>

#include <easy/profiler.h>

#include "IMUExtrinsicCalibrator.h"
#include <gtsam/geometry/Rot3.h>

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

gtsam::Rot3 getRandomRotation()
{
    float roll = float( rand() % 180 - 90 );
    float pitch = float( rand() % 180 - 90 );
    float yaw = float( rand() % 360 - 180 );

    return gtsam::Rot3(gtsam::Rot3::Rz(deg2rad(yaw)) * gtsam::Rot3::Ry(deg2rad(pitch)) * gtsam::Rot3::Rx(deg2rad(roll)));
}

gtsam::Rot3 getNoiseRotation()
{
    float roll = float( rand() % 200 - 100 ) / 100.f;
    float pitch = float( rand() % 200 - 100 ) / 100.f;
    float yaw = float( rand() % 200 - 100 ) / 100.f;

    return gtsam::Rot3(gtsam::Rot3::Rz(deg2rad(yaw)) * gtsam::Rot3::Ry(deg2rad(pitch)) * gtsam::Rot3::Rx(deg2rad(roll)));
}

int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);
    profiler::startListen();
    std::cout << "GTSAM Testbench" << std::endl;

    IMUExtrinsicCalibrator calibrator(true, true);

    std::string value;
    std::ifstream file("~/out.csv");
    if (!file.is_open()) return 0;

    double QuatValues[8];
    while (file.good()) {
        for (int i = 0; i < 7; ++i) {
            getline(file, value, ',');
            QuatValues[i] = atof(value.c_str());
            if (!file.good()) break;
        }
        if (!file.good()) break;
        getline(file, value, '\n');
        QuatValues[7] = atof(value.c_str());
        if (!file.good()) break;

        gtsam::Rot3 R_realsense = gtsam::Rot3::Quaternion(QuatValues[0], QuatValues[1], QuatValues[2], QuatValues[3]);
        gtsam::Rot3 R_imu = gtsam::Rot3::Quaternion(QuatValues[4], QuatValues[5], QuatValues[6], QuatValues[7]);
        calibrator.AddMeasurement(R_imu, R_realsense);
    }

    GTSAM_PRINT(calibrator.LM_Optimize());
}
