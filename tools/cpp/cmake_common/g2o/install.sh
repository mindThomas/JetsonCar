#!/bin/bash

set -e  # exit on error

# Install dependencies
sudo apt-get install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev -y

TMP_PATH=/tmp
G2O_PATH=${TMP_PATH}/g2o
G2O_BUILD_PATH=${G2O_PATH}/build

if [ ! -d ${G2O_PATH} ] ; then # G2O has not been downloaded does not exist
	echo "Cloning G2O"
	cd ${TMP_PATH}
	git clone https://github.com/RainerKuemmerle/g2o
	cd ${G2O_PATH}
	git checkout fcba4ea # November 2019 commit
fi

if [ ! -d ${G2O_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${G2O_BUILD_PATH}
fi

exit -1

# Create makefiles and build
echo "Building G2O library"
cd ${G2O_BUILD_PATH}
cmake ..
make -j4

BUILD_PATH=${G2O_PATH}/lib/libg2o_core.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull G2O build"
	exit -1
fi

# Install library and headers
echo "Installing G2O library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${G2O_PATH}
