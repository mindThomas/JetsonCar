#!/bin/bash

set -e  # exit on error

# Install dependencies
sudo apt install build-essential pkg-config cmake libwxgtk3.0-dev libwxgtk3.0-gtk3-dev libopencv-dev libeigen3-dev libgtest-dev
sudo apt install libftdi-dev freeglut3-dev zlib1g-dev libusb-1.0-0-dev libudev-dev libfreenect-dev libdc1394-22-dev libavformat-dev libswscale-dev libassimp-dev libjpeg-dev   libsuitesparse-dev libpcap-dev liboctomap-dev
# Add ROS support
sudo apt install libcv-bridge-dev libgeometry-msgs-dev libnav-msgs-dev librosbag-dev libroscpp-dev libsensor-msgs-dev libstd-srvs-dev libstereo-msgs-dev libtf2-dev libtf2-msgs-dev libbz2-dev

TMP_PATH=/tmp
MRPT_PATH=${TMP_PATH}/mrpt
MRPT_BUILD_PATH=${MRPT_PATH}/build

if [ ! -d ${MRPT_PATH} ] ; then # MRPT has not been downloaded does not exist
	echo "Cloning MRPT"
	cd ${TMP_PATH}
	git clone https://github.com/MRPT/mrpt.git
	cd ${MRPT_PATH}
	git checkout f67d0f8 # release 1.5.8
fi

if [ ! -d ${MRPT_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${MRPT_BUILD_PATH}
fi

# Create makefiles and build
echo "Building MRPT library"
cd ${MRPT_BUILD_PATH}
cmake ..
make

BUILD_PATH=${MRPT_BUILD_PATH}/src/libmrpt.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull MRPT build"
	exit -1
fi

# Install library and headers
#echo "Installing MRPT library"
#sudo make install
#sudo ldconfig

# Remove temp files
#echo "Cleaning up"
#rm -rf ${MRPT_PATH}
