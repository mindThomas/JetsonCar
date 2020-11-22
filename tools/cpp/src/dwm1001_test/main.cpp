#include "decawave/dwm1001.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

void MeasurementsCallback(DWM1001::TagToAnchorMeasurements m)
{
    std::cout << "Measurements:" << std::endl;
    for (auto&& meas : m.measurements) {
        std::cout << "ID #" << meas.first << " = " << meas.second.range << std::endl;
    }
    if (m.estimate.valid) {
        std::cout << "Estimated position: (" << m.estimate.position[0] << ", "
                                             << m.estimate.position[1] << ", "
                                             << m.estimate.position[2] << ")"
                                             << std::endl;
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);
    std::cout << "Decawave DWM1001 Test" << std::endl;

    DWM1001 dwm1001;
    dwm1001.Connect();
    dwm1001.EnterStreamMode();
    dwm1001.RegisterMeasurementCallback(MeasurementsCallback);

    while (!ShouldExit)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    dwm1001.Disconnect();
}