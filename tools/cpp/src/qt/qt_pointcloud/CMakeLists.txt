# Edit the configuration line below to match binary needs
set(binary_dependencies    Qt5::Widgets  easy_profiler
    QGLVisualizer
    ${PCL_LIBRARIES}
    jetsoncar_utils)

# Configure build as release (for maximum optimization) 
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_WITH_EASY_PROFILER")

prepare_qt_binary()

# But still add debug information (-g flag)
#set_target_properties( ${folder_name} PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra" )
