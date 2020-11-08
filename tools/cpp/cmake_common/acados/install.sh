#!/bin/bash

set -e  # exit on error

GIT_URL=https://github.com/acados/acados.git
GIT_HASH=993c92291af17a787865ce36167e2e1011d75d3e

CLONE_DIRECTORY=~/repos
REPO_FOLDER=acados
ACADOS_PATH=${CLONE_DIRECTORY}/${REPO_FOLDER}
ACADOS_BUILD_PATH=${ACADOS_PATH}/build

if [ ! -d ${ACADOS_PATH} ] ; then # ACADOS has not been downloaded does not exist
    echo "Cloning acados"
    mkdir -p ${CLONE_DIRECTORY}
    cd ${CLONE_DIRECTORY}
    git clone ${GIT_URL} ${REPO_FOLDER}
else
    read -p "acados has already been downloaded. Do you want to delete it and clone again? (y/n)?" choice
    case "$choice" in 
      y|Y ) echo "Deleting acados folder"
            rm -rf ${ACADOS_PATH}
            cd ${CLONE_DIRECTORY}
            git clone ${GIT_URL} ${REPO_FOLDER};;
      n|N ) echo "Will not delete acados folder but checkout necessary version and recompile";;
      * ) echo "Cancelling"
          exit -1;;
    esac
fi

cd ${ACADOS_PATH}    
git checkout ${GIT_HASH}
git submodule update --recursive --init

if [ ! -d ${ACADOS_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${ACADOS_BUILD_PATH}
fi

# Create makefiles and build
echo "Building acados library"
cd ${ACADOS_BUILD_PATH}

cmake -DCMAKE_BUILD_TYPE="Release" -DACADOS_PYTHON=ON -DACADOS_WITH_QPOASES=ON ..
# -DBLASFEO_TARGET= GENERIC / X64_INTEL_HASWELL / ARMV8A_ARM_CORTEX_A57    (see https://github.com/giaf/blasfeo/blob/master/README.md)
# -DHPIPM_TARGET=
make install

BUILD_PATH=${ACADOS_PATH}/lib/libacados.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull acados build"
	exit -1
fi

# Install library and headers
#echo "Installing ACADOS library"
#sudo make install
#sudo ldconfig

# Remove temp files
#echo "Cleaning up"
#rm -rf ${ACADO_PATH}
