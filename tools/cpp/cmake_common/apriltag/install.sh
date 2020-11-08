#!/bin/bash

set -e  # exit on error

# Install dependencies
#sudo apt-get install cmake libopencv-dev libeigen3-dev libv4l-dev
sudo apt-get install libv4l-dev

TMP_PATH=/tmp
APRILTAGS_PATH=${TMP_PATH}/apriltag
APRILTAGS_BUILD_PATH=${APRILTAGS_PATH}/build

if [ ! -d ${APRILTAGS_PATH} ] ; then # Pangolin has not been downloaded does not exist
	echo "Cloning Apriltag"
	cd ${TMP_PATH}
	git clone https://github.com/AprilRobotics/apriltag.git
	cd ${APRILTAGS_PATH}
fi

if [ ! -d ${APRILTAGS_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${APRILTAGS_BUILD_PATH}
fi

# Create makefiles and build
echo "Building Apriltag library"
cd ${APRILTAGS_BUILD_PATH}
cmake ..
make -j8

BUILD_PATH=${APRILTAGS_BUILD_PATH}/lib/libapriltags.a

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull Apriltag build"
	exit -1
fi

# Install library and headers
echo "Installing Apriltag library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${APRILTAGS_PATH}
