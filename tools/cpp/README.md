# JetsonCar -- C++ tools
Code, libraries and other tools built for the Jetson RC car project but does not fit in the other repositories, eg. log-processing, periphiral drivers/interfaces, startup scripts etc.

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