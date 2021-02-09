# JetsonCar -- Python tools
Code, libraries and other tools built for the Jetson RC car project but does not fit in the other repositories, eg. MPC code generators, log-processing etc.

## Environment setup instructions
1. Install Anaconda or Miniconda
2. Build the `cpp` folder in `tools` to download and install binary depdencies (e.g. `acados`).
2. Create the environment `conda create -y --name jetsoncar python=3.7`
     1. Alternatively based on the environment file
`conda env create environment.yml -n jetsoncar`
3. Activate the environment
`conda activate jetsoncar`
4. Install `acados` into environment: `pip3 install -e ~/repos/acados/interfaces/acados_template`
5. Install other libraries listed below:
```
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag
pip install lz4
pip install bagpy --user
pip install -U py3rosmsgs
pip install git+https://github.com/vioshyvo/mrpt/
pip install roboticstoolbox-python
pip install cvxpy
pip install casadi
```
4. Install libraries from the requirements file (can be omitted if the environment-file based setup is used: `pip3 install -r requirements.txt`
    4.1 Alternatively `conda install --force-reinstall -y -q --name jetsoncar -c conda-forge --file requirements.txt`
4. Configure the environment to use the libraries (setting path variables etc.). It is important to run this command from the same directory as where it lives.
`source conda_environment_setup.sh`


## Notes

### Usage with PyCharm
When using this with PyCharm you need to launch PyCharm from a console after activating the environment. Furthermore you should point the Interpreter settings to the conda enviroment within PyCharm.