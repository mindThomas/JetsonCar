# JetsonCar -- C++ tools
Code, libraries and other tools built for the Jetson RC car project but does not fit in the other repositories, eg. log-processing, periphiral drivers/interfaces, startup scripts etc.

## Prerequisites
Other prerequisites that are not installed automatically:
```
sudo apt-get install libi2c-dev i2c-tools
```

### ROS prerequisites
If you want to use the ROS parts of the tools, ROS will have to be sourced when building and you will also have to install the following dependencies:
```
sudo apt-get install ros-$ROS_DISTRO-rosbag
```

## How to build
```bash
mkdir build
cd build
cmake ..
make
```

## Notes
Install `cmake-qt-gui` to configure CMakeLists projects interactively.

cmake v3.10 or later is required due to GTSAM