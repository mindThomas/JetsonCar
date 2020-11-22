#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>

#include <easy/profiler.h>
#include "imu_spatial/imu_spatial.h"

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}



int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);
    profiler::startListen();
    std::cout << "IMU Test" << std::endl;

    size_t count = 0;
    IMU imu;

    imu.Connect();
    //imu.Configure(IMU::OutputType::Estimates, 100);
    imu.Configure(IMU::OutputType::RawSensors, 100);

    while (!ShouldExit)
    {
        /*if (imu.angular_velocity_received_ && imu.raw_sensors_received_)
        {
            imu.quaternion_orientation_std_received_ = false;
            imu.quaternion_orientation_received_ = false;
            imu.acceleration_received_ = false;
            imu.angular_velocity_received_ = false;

            printf("Angular velocity (X,Y,Z) = %2.3f, %2.3f, %2.3f rad/s\n",
                    imu.angular_velocity_packet_.angular_velocity[0],
                    imu.angular_velocity_packet_.angular_velocity[1],
                    imu.angular_velocity_packet_.angular_velocity[2]
                  );
        }*/
        //imu.GetStatus();

        auto angular_velocity = imu.GetGyroscope();
        //auto estimate = imu.GetEstimate();

        printf("Angular velocity (X,Y,Z) = [%2.3f] %2.3f, %2.3f, %2.3f rad/s\n",
               angular_velocity.timestamp,
               angular_velocity.measurement[0],
               angular_velocity.measurement[1],
               angular_velocity.measurement[2]
        );

        /*printf("RPY = [%2.3f] %2.3f, %2.3f, %2.3f deg\n",
               estimate.timestamp,
               rad2deg(estimate.RPY[0]),
               rad2deg(estimate.RPY[1]),
               rad2deg(estimate.RPY[2])
        );*/

        /*printf("XYZ = %2.3f, %2.3f, %2.3f m\n",
               estimate.position[0],
               estimate.position[1],
               estimate.position[2]
        );*/


        if ((count++ % 100) == 0) {
            //imu.SetHeading(deg2rad(0), deg2rad(2));
            imu.SetPositionNED(0, 0, 0, 1);
            imu.GetStatus();
        }

        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    imu.Disconnect();
}
