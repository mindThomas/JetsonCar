#!/bin/bash

set -e  # exit on error

## See https://github.com/MRPT/mrpt

# Install v1.5 from PPA
sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5
sudo apt-get update
sudo apt-get install libmrpt-dev mrpt-apps -y

# Install v2.0 from PPA
#sudo add-apt-repository ppa:joseluisblancoc/mrpt
#sudo apt-get update
#sudo apt-get install libmrpt-dev mrpt-apps
