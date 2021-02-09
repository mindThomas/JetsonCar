# JetsonCar -- ROS

## Prerequisites
1. Install Ubuntu 18.04 through JetPack 4.4.1 on Jetson TX2.
2. Configure the PC name to be: `jetson-car`
3. Configure the username to be: `jetson`
4. Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
5. Optionally the [QT Creator for ROS](https://ros-qtc-plugin.readthedocs.io/en/latest/index.html) can be installed


## Setup and Build
It is assumed that you have already cloned the [`JetsonCar`](https://github.com/mindThomas/JetsonCar/) repository as described in the root 
[`README.md`](https://github.com/mindThomas/JetsonCar/blob/master/README.md).

### Package prerequisites
This project uses ROS Melodic and the following extra dependencies:
```bash
sudo apt-get install python-catkin-tools
sudo apt-get install python-rosdep
sudo apt install ros-melodic-rtabmap ros-melodic-rtabmap-ros
sudo apt install ros-melodic-ackermann-msgs
sudo apt install ros-melodic-octomap*
sudo apt-get install ros-melodic-realsense2-camera ros-melodic-realsense2-description
sudo apt install ros-melodic-plotjuggler-ros
sudo apt-get install libqwt-dev
sudo apt-get install ros-melodic-rqt-multiplot
```

### Setup ROS workspace
To set up the ROS workspace we create a new catkin workspace. Follow the steps below to set up a new catkin workspace and create symlink to the cloned repository:
```bash
# Create a workspace folder if one doesn't exist
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# Initialize as catkin workspace
catkin_init_workspace

# Create symlinks to necessary folders from the cloned repository
ln -s <path to JetsonCar repo>/ros ./jetsoncar
ln -s <path to JetsonCar repo>/lib/cpp ./lib

# Cloning dependencies
# 1. Realsense T265 ROS wrapper
git clone --single-branch --branch thomas/development https://github.com/mindThomas/realsense-ros

# 2. ethz-asl libraries and nodes
git clone https://github.com/ethz-asl/image_undistort.git

# 3. ROSbag conversion tool
git clone https://github.com/AtsushiSakai/rosbag_to_csv.git

# 4. Decawave DWM1001 (MDEK1001) node
git clone https://github.com/mindThomas/dwm1001_ros.git

# 5. ROS Message Parser
git clone https://github.com/facontidavide/ros_msg_parser.git

# Install automatic dependencies
cd .. # go back to ros_ws folder
rosdep install --from-paths src --ignore-src -r -y
```

### Building
Build the project with catkin build
```bash
cd ~/ros_ws

# Build the packages
catkin build

# Source local ROS setup file (use setup.zsh if you are on Zsh)
source devel/setup.bash
```

### Configure and use with CLion
See also https://www.jetbrains.com/help/clion/ros-setup-tutorial.html

Initial setup steps
1. With the `~/ros_ws` folder create a folder named: `clion`
2. Create a folder in `clion` named: `cmake-build`
3. You should now have `~/ros_ws/clion/cmake-build`
4. Open CLion normally and open the CMakeLists as a project: `~/ros_ws/src/CMakeLists.txt`
5. Open the CLion settings and change the CMake settings:
     1. Set the Build type to Default
     2. Set the Generation Path to: `~/ros_ws/clion/cmake-build`
6. Reload the CMakeLists by right-clicking on the top-most `src` folder
7. Compile the full project and accept that it might fail with some libraries
8. Exit CLion and proceed with the launch steps below

CLion launch steps
1. Open a terminal and source the CLion path specific `setup.bash` from: `~/ros_ws/clion/cmake-build/devel/setup.bash`
2. Launch CLion from the same terminal
3. Now everything should build and compile properly

# SLAM
Which SLAM system/library to use?
Performance comparison tool: https://michaelgrupp.github.io/evo/

## Existing ROS SLAM libraries
A list of Open Source libraries: https://www.rsj.or.jp/databox/international/iros16tutorial_2.pdf (see in the end of the slide deck)
SLAM benchmark: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
Visual SLAM: https://www.youtube.com/watch?v=ymI3FmwU9AY

 - [Google Cartographer](https://google-cartographer.readthedocs.io/en/latest/)
 - GMapping
 - Hector SLAM
 - [RTAB-Map](http://wiki.ros.org/rtabmap_ros) (Stereo/RGBD only VO and SLAM) - see https://www.youtube.com/watch?v=tcJHnHpwCXk and launch file here: https://github.com/introlab/rtabmap_ros/blob/master/launch/rtabmap.launch or tutorial here: http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
 - [ORB-SLAM 2](https://github.com/raulmur/ORB_SLAM2) (Monocular, Stereo or RGBD)
 -- ORB-SLAM 2 as pure Visual Odometry: https://github.com/raulmur/ORB_SLAM2/issues/256
 - [ROVIO](https://github.com/ethz-asl/rovio)
 - [OKVIS](https://github.com/ethz-asl/okvis)
 - [SVO](https://github.com/uzh-rpg/rpg_svo)
 - [DSO](https://github.com/JakobEngel/dso)
 - [Stereo DSO](https://github.com/JiatianWu/stereo-dso)
 - [LSD-SLAM](https://github.com/tum-vision/lsd_slam)
 - DTAM
 - [LAMA](https://github.com/iris-ua/iris_lama) - see https://msadowski.github.io/iris-lama-slam-with-ros/
 - [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - see https://msadowski.github.io/hands-on-with-slam_toolbox/
 - [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
 
## Feature Extractors
Binary descriptors (ORB, BRIEF, FREAK) are more suitable for odometry, for which we want very fast extraction/matching. Float descriptors (SIFT, SURF) are more suitablefor loop closure detection, because they are more discriminative (at higher computation cost).

### ORB
ORB addresses the detection, the orientation assignment and the descriptor extraction phases rather than only the descriptor (i.e. BRIEF). ORB (Oriented FAST and Rotated BRIEF) adopts a multi-scale approach, where the FAST detector is run independently for each layer of a Gaussian pyramid. Thus, for each detected key-points the Harris response (instead of the FAST response) and the orientation angle are computed. The orientation is estimated using the Intensity Centroid method. The Harris response helps to retain the best N key-points in adaptive way over the scale layers. The BRIEF sampling pattern (pairs of pixels for the intensity comparison within the patch) to built the descriptor is randomly selected from a Gaussian distribution (even if the original paper investigates 5 types of selection). Unlike BRIEF, ORB learnt the sampling pattern to achieve high variance and low correlation between the tests for a patch size 31x31. If using a different patch size, the ORB sampling pattern will be random. Before extracting the descriptor, the estimated orientation angle is applied to the ORB sampling pattern.
 
## Loop closure detection
RTAB-Map example: https://github.com/introlab/rtabmap/wiki/Cplusplus-Loop-Closure-Detection
 
# Map
## 2D Occupancy grid

## Octomap
See https://www.youtube.com/watch?v=yKNzTg25RM8 which uses `octomap_server`
```
<?xml version="1.0"?>
 
<launch>
 
    <node pkg="octomap_server" type="octomap_server_node" name="octomap_server" output="screen">
          <param name="resolution" value="0.1" />
          <param name="base_frame_id" value="zed_center" />
          <remap from="cloud_in" to="/zed/point_cloud/cloud_registered" />
    </node>
 
    <node name="rviz" pkg="rviz" type="rviz" output="screen" />
 
</launch>
```
See also: https://github.com/RyuYamamoto/turtlebot_octomap

# Additional information

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [Catkin tutorial](http://wiki.ros.org/catkin/Tutorials)
- [Stereolab ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper)


# Developer notes

Descriptions and guides how to use this ROS project can be found in the notes below.

## USB rules file for automatic device detection
The MCU can automatically be detected and assigned to `/dev/jetsoncar` when connecting it over USB if the rules file, `99-jetsoncar.rules`, is installed.

To install the rules file, copy `99-jetsoncar.rules` to `/etc/udev/rules.d/`.


## Install as service on boot
A startup script and corresponding service (for starting at boot) for launching the minimal bringup launch file has been made.

Copy the `startup_launch.sh` and `PrepareHostROS.sh` to the home folder. Copy the file `jetsoncar.service` into `/lib/systemd/system` and modify the path to the `startup_launch.sh` script accordingly and rename the user and group to the username on the device if different. Enable the service on boot by running:
```bash
sudo systemctl daemon-reload
sudo systemctl enable jetsoncar.service
```

After the service has been installed the driver can be started, stopped or restarted by using:
```bash
sudo service jetsoncar start
sudo service jetsoncar stop
sudo service jetsoncar restart
```

## ZED Camera
The ZED launch file is currently configure to only output the rectified depth image (and pointcloud if needed) but not without the position tracking enabled (Visual Odometry or SLAM) since this would consume unnecessary CPU ressources.
However, if any node subscribes to the `/zed/zed_node/pose` or `/zed/zed_node/odom` topics, the position tracking will automatically be enabled and the CPU load will increase. See more: https://github.com/stereolabs/zed-ros-wrapper/issues/438

## T265 Tracking camera
The two stereo fiheye image streams can also be undistorted and used for other purposes (even SLAM). It is however complicated to do the undistortion out of the box since the parameters and the distortion model does not work with the [`image_proc`](http://wiki.ros.org/image_proc) nodes. Instead the nodes within the `image_undistort` package by __ethz-asl__ can be used.
Alternatively the code in our realsense driver (see the C++ common libraries) also includs the code for stereo rectification.
See also: http://official-rtab-map-forum.67519.x6.nabble.com/Slam-using-Intel-RealSense-tracking-camera-T265-td6333.html#a6343

## XSens IMU
The following ROS library/node is used: http://wiki.ros.org/xsens_mti_driver
Remember to enable low latency serial mode (https://base.xsens.com/hc/en-us/community/posts/360029341694-How-do-I-get-IMU-data-via-USB-in-real-time-?page=1#community_comment_360004975314):
```bash
sudo apt-get install setserial
setserial /dev/ttyXsens low_latency
```

## Recording a log
To record a log execute the following command preferably on the onboard computer running the drivers to limit latency:
```
rosbag record -a -o /path/to/file.bag
```

## Record certain topics
Use the `rosbag` tool:
```
rosbag record /tf /camera/odom/sample
```

Using a RegEx expression we can select the topics based on wildcards:
```
rosbag record -e "/tf|/camera/odom/(.*)|/camera/accel/(.*)|/camera/gyro/(.*)"
```

## Playing back a log
```
rosparam set use_sim_time true
```
Or in the Launch file
```
<param name ="/use_sim_time" value="true"/>
```

Without GUI:
```
rosbag play --clock <log.bag>
```
With GUI
```
rosrun rqt_bag rqt_bag --clock
```
Note that the GUI does not seem to be able to handle the transform publication correctly.

Alternatively the following launch file can be used:
```
roslaunch jetsoncar_launch rosbag.launch bag:="log_name.bag"
```

## ROS bag to CSV
Either it can be done directly using the rosbag functionality of the rostopic echo tool:
```
rostopic echo -b <log.bag> -p /tf > tf.csv
```
This will result in a CSV file with the first row defining the column names.
To extract individual CSV files for each topic the following one-lines can be used:
```
for topic in `rostopic list -b bagfile.bag` ; do rostopic echo -p -b bagfile.bag $topic >bagfile-${topic//\//_}.csv ; done
```

Alternatively the `rosbag_to_csv` tool that we have cloned can be used.
```
rosrun rosbag_to_csv rosbag_to_csv.py
```

ROSbags can also be investigated using
```
rosrun rqt_bag rqt_bag <log.bag>
```

## Quick browse of ROS bag data
Finally The topics and their message types contained within a ROSbag can be opened and investigated with the Realsense [rosbag-inspector](https://github.com/IntelRealSense/librealsense/tree/master/tools/rosbag-inspector) from librealsense.
```
rs-rosbag-inspector
```

## Enable debug message
Verbosity of the ROS console message is configured through the debug logger. By default `ROS_DEBUG` and `ROS_DEBUG_STREAM` is not printed to `stdout` (in the terminal).

To enable this one can either include a `rosconsole.conf` in a launch file, similar to how it is done in `jetson_driver` or execute the following command after launching the node:
```
rosservice call /<node name>/set_logger_level "{logger: 'ros', level: 'debug'}" 
```

Alternatively debug verbosity can also be enabled programatically with:
```
if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();
```

## List content of particular topic
Automatic look up of message type based on channel/topic (here exampe with the channel `/xsens/imu/acceleration`):
```bash
rosmsg info $(rostopic info /xsens/imu/acceleration | grep "Type:" | grep -oe "\w*\/\w*")
```

## Plotting data
PlotJuggler is a strong GUI tool that can be used to plot data both from ROS bags, CSV files etc.
https://github.com/facontidavide/PlotJuggler
Install using snap:
```
sudo snap install plotjuggler
```

Alternatively [rqt_multiplot](https://github.com/ANYbotics/rqt_multiplot_plugin) can be used with better configuration freedom than the built-in `rqt_plot`.
```
rosrun rqt_multiplot rqt_multiplot
```

## Processing ROS bag data with MATLAB, Python and C++
**Native rosbag (C++)**
- C++ API: http://wiki.ros.org/rosbag/Code%20API
- http://wiki.ros.org/rosbag
- https://github.com/ros/ros_comm/tree/kinetic-devel/tools/rosbag/scripts
- http://wiki.ros.org/rosbag/Cookbook 
- http://docs.ros.org/kinetic/api/rosbag/html/index.html

**bag tools**
- http://wiki.ros.org/bag_tools

**rqt_bag**
- http://wiki.ros.org/rqt_bag
- http://wiki.ros.org/rqt_bag_plugins

**rtq_bag_exporter**
- https://gitlab.com/InstitutMaupertuis/rqt_bag_exporter
- https://discourse.ros.org/t/qt-rosbag-exporter/4964

**MARV Robotics (extensible data management platform)**
- Code: https://github.com/ternaris/marv-robotics
- Example: https://marvhub.com/#/collection/bags
- Example: https://marvhub.com/#/detail/q4mwxc3epkcqpqhiyoru7xclii

**MATLAB solutions**
- Official ROS toolbox (not free): https://www.mathworks.com/help/ros/ref/rosbag.html
- (tested and working well): https://github.com/unl-nimbus-lab/bag2matlab
- (untested): https://github.com/bcharrow/matlab_rosbag (This library wraps the C++ rosbag API and therefore it supports bag file format 1.2 and later, including things like reading compressed messages. Message instances are converted to MATLAB structs using the message definitions contained within the bag.)

**Various bag tools (trimming, merging, editing, ... bag files)**
- bagtools: https://bitbucket.org/daniel_dube/bagedit/wiki/Home
- Rosbag Editor: https://github.com/facontidavide/rosbag_editor
- rosbag_fixer (work around message header's missing dependency information): https://github.com/gavanderhoorn/rosbag_fixer