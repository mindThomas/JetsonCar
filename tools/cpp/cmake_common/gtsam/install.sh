#!/bin/bash

set -e  # exit on error

# Install dependencies
# ...

TMP_PATH=/tmp
GTSAM_PATH=${TMP_PATH}/gtsam
GTSAM_BUILD_PATH=${GTSAM_PATH}/build

if [ ! -d ${GTSAM_PATH} ] ; then # GTSAM has not been downloaded does not exist
	echo "Cloning GTSAM"
	cd ${TMP_PATH}
	git clone https://github.com/borglab/gtsam.git
	cd ${GTSAM_PATH}
	git checkout 52e8db6 # release 4.0.0
fi

if [ ! -d ${GTSAM_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${GTSAM_BUILD_PATH}
fi

# Create makefiles and build
echo "Building GTSAM library"
cd ${GTSAM_BUILD_PATH}
cmake -DCMAKE_BUILD_TYPE=Release ..
make check -j3

BUILD_PATH=${GTSAM_BUILD_PATH}/gtsam/libgtsam.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull GTSAM build"
	exit -1
fi

# Install library and headers
echo "Installing GTSAM library"
sudo make install -j3
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${GTSAM_PATH}
