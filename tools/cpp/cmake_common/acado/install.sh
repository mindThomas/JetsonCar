#!/bin/bash

set -e  # exit on error

TMP_PATH=/tmp
ACADO_PATH=${TMP_PATH}/ACADOtoolkit
ACADO_BUILD_PATH=${ACADO_PATH}/build

if [ ! -d ${ACADO_PATH} ] ; then # ACADO has not been downloaded does not exist
	echo "Cloning ACADO"
	cd ${TMP_PATH}
	git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
	cd ${ACADO_PATH}
	git checkout b4e28f3131f79cadfd1a001e9fff061f361d3a0f
fi

if [ ! -d ${ACADO_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${ACADO_BUILD_PATH}
fi

# Create makefiles and build
echo "Building ACADO library"
cd ${ACADO_BUILD_PATH}
cmake -DCMAKE_BUILD_TYPE="Release" ..
make

BUILD_PATH=${ACADO_BUILD_PATH}/lib/libacado_toolkit_s.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull ACADO build"
	exit -1
fi

# Configure environment variables for CMake to use
source acado_env.sh

# Install library and headers
echo "Installing ACADO library"
#sudo make install
#sudo ldconfig

# Remove temp files
#echo "Cleaning up"
#rm -rf ${ACADO_PATH}
