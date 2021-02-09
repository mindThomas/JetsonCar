cmake_minimum_required(VERSION 3.5)

project(jetsoncar_libraries)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO) # use -std=c++14 instead of -std=gnu++14

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
## Find Boost
find_package(Boost REQUIRED COMPONENTS system signals python thread program_options regex timer QUIET)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(CURL REQUIRED)
find_package(easy_profiler)

find_package(realsense2 REQUIRED)

find_package(gflags REQUIRED)

set(OPENCV_SEARCH_PATH "~/repos/opencv-4.3.0/install/")
list(APPEND CMAKE_MODULE_PATH "${OPENCV_SEARCH_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${OPENCV_SEARCH_PATH}")
message(STATUS "OpenCV search path: ${OPENCV_SEARCH_PATH}")
find_package(OpenCV 4.3 REQUIRED)

## Qt5 modules needed by OpenCV
find_package(Qt5
      COMPONENTS        
        Core        
        Gui        
        Widgets
        Test
        Concurrent
        OpenGL
      REQUIRED)

## PCL
find_package(PCL REQUIRED COMPONENTS common search features filters keypoints registration QUIET)
#find_package(PCL REQUIRED COMPONENTS common search features visualization QUIET)  # this line is needed to use PCL Visualizer
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# cherry pick the only module in io to prevent conflicts
# of its components, such as vtk with other nucore dependencies
set(PCL_LIBRARIES ${PCL_LIBRARIES} ${PCL_LIBRARY_DIRS}/libpcl_io.so)

# these lines are needed to use PCL Visualizer
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
#set(PCL_LIBRARIES ${PCL_LIBRARIES} vtkCommonCore-6.2.so )
#include_directories(/usr/include/vtk)
#include_directories(/usr/lib64/vtk)

find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

mark_as_advanced(
        FLANN_INCLUDE_DIRS
        FLANN_LIBRARY
        FLANN_LIBRARY_DEBUG
        PCL_COMMON_INCLUDE_DIR
        PCL_DIR
        PCL_FEATURES_INCLUDE_DIR
        PCL_FILTERS_INCLUDE_DIR
        PCL_KDTREE_INCLUDE_DIR
        PCL_OCTREE_INCLUDE_DIR
        PCL_SAMPLE_CONSENSUS_INCLUDE_DIR
        PCL_SEARCH_INCLUDE_DIR
        PCL_KEYPOINTS_INCLUDE_DIR
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
	
## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

# Hack to get to the ros_ws/src/lib folder created as a symlink (see README.md)
set(LIB_DIR "${CATKIN_DEVEL_PREFIX}/../../../src/lib")
set(EXCLUDE_DIRS spdlog)

# Include libraries
MACRO(CMAKE_COMMON_LIBS_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list ${LIB_DIR}/CMakeLists.txt)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)         
        FOREACH(EXCLUDE_DIR ${EXCLUDE_DIRS})
            string (FIND ${dir_path} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)  
            if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
                break()
            endif()
        ENDFOREACH()        
        if (${EXCLUDE_DIR_FOUND} EQUAL -1)
            SET(dir_list ${dir_list} ${dir_path})
        endif()
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})    
ENDMACRO()

CMAKE_COMMON_LIBS_DIRECTORIES(SUBDIRS)
set(JETSONCAR_LIBRARIES "")
FOREACH(subdir ${SUBDIRS})
  get_filename_component(subdir_name "${subdir}" NAME)
  #set(JETSONCAR_LIBRARIES ${JETSONCAR_LIBRARIES} ${subdir_name})
  list(APPEND JETSONCAR_LIBRARIES ${subdir_name})
ENDFOREACH()

if (NOT ZED_FOUND)
    list(REMOVE_ITEM JETSONCAR_LIBRARIES "zed") # not used for now by the ROS subsystem
endif()

# Add SPDLOG
option(SPDLOG_BUILD_SHARED "" OFF)
add_subdirectory(${LIB_DIR}/spdlog  "lib/spdlog")
#get_target_property(SPDLOG_INCLUDE_DIRS spdlog INCLUDE_DIRECTORIES)
get_target_property(SPDLOG_SOURCE_DIR spdlog SOURCE_DIR)
set(SPDLOG_INCLUDE_DIR ${SPDLOG_SOURCE_DIR}/include)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${LIB_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${SPDLOG_INCLUDE_DIR}
  LIBRARIES ${JETSONCAR_LIBRARIES} # easy_profiler realsense2
  CATKIN_DEPENDS roscpp
  DEPENDS Boost OpenCV CURL EIGEN3 GFLAGS PCL
)
## OBS! The order is important. 'catkin_package' has to come before all 'add_executable' and 'add_library'!

include_directories(${LIB_DIR})

message(STATUS "Adding common libraries:")
FOREACH(subdir ${SUBDIRS})
  get_filename_component(subdir_name "${subdir}" NAME)
  #message(STATUS "   " ${subdir_name})
  ADD_SUBDIRECTORY(${subdir} "lib/${subdir_name}")  
ENDFOREACH()

#message(FATAL_ERROR "${JETSONCAR_LIBRARIES}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${LIB_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/jetsoncar_test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/jetsoncar_communication_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#   scripts/teleop.py
#   scripts/keyboard_teleop.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS ${JETSONCAR_LIBRARIES}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#install(
#    DIRECTORY cfg config launch
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jetsoncar_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
