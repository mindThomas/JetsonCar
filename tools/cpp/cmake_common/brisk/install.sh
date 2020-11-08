#!/bin/bash

set -e  # exit on error

# Install dependencies

TMP_PATH=/tmp
BRISK_PATH=${TMP_PATH}/brisk
BRISK_BUILD_PATH=${BRISK_PATH}/build

if [ ! -d ${BRISK_PATH} ] ; then # Pangolin has not been downloaded does not exist
	echo "Cloning BRISK"
	cd ${TMP_PATH}
	git clone https://github.com/mindThomas/brisk.git
	cd ${BRISK_PATH}
fi

if [ ! -d ${BRISK_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${BRISK_BUILD_PATH}
fi

# Create makefiles and build
echo "Building BRISK library"
cd ${BRISK_BUILD_PATH}
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

BUILD_PATH=${BRISK_BUILD_PATH}/lib/libbrisk.a

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull BRISK build"
	exit -1
fi

# Install library and headers
echo "Installing BRISK library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${BRISK_PATH}
