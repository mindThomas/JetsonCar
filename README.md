# JetsonCar
This is the monolithic repository for the Jetson RC car project including embedded firmware, ROS nodes, libraries, MATLAB scripts, startup scripts etc.
This repository collates the work from multiple repositories related to the JetsonCar project to simplify version control. The other individual repositories have therefore been deprecated.

## Folder structure
1. `firmware` : Embedded firmware for the STM32F4 board on the Jetson Car for communication with ROS driver, parsing XV11 LiDAR data, reading Quadrature signal from encoders, generating PWM output for DC motors. 
2. `lib/cpp` : C++ libraries used by both ROS nodes and the C++ tools.
3. `lib/matlab` : MATLAB libraries used by the MATLAB tools.
4. `lib/python` : Python libraries used by both ROS nodes and the Python tools.
5. `ros` : ROS Nodes including LiDAR, GPS, IMU processing, SLAM, Sensor fusion, controllers and possibly Machine learning.
6. `simulation` : Simulation environment for the Jetson Car project using Gazebo and ROS.
7. `tools/cpp` : C++ tools built for the Jetson RC car project which is not related to the ROS nodes, eg. log-processing, periphiral drivers/interfaces, startup scripts etc.
8. `tools/matlab` : MATLAB code for the Jetson RC car project including Motor system identification, controller design and state estimator design.
9. `tools/python` : Python tools and scripts for visualization, post processing, data analysis etc.

# Cloning
If you have not already cloned this repository you do so by using the following command:
`git clone --recursive https://github.com/mindThomas/JetsonCar.git`

If you have already cloned the repository but without the submodules (not recursively), you will have to initialize the submodules with:
`git submodule update --init --recursive`

# Installation
See the `README.md` files in the respective folders such as `ros`, `simulation` and `tools/cpp`.
