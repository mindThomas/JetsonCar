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
