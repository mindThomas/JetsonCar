# Edit the configuration line below to match binary needs
set(binary_dependencies    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Xml Qt5::PrintSupport
    Qt5::OpenGL
    Qt5::3DCore Qt5::3DRender Qt5::3DInput Qt5::3DExtras
    ${QGLVIEWER_LIBRARY}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    QGLVisualizer
    realsense    
    imu    
    utils
    )

if (${ZED_FOUND})    
    list(APPEND binary_dependencies zed)
endif()

# Configure build as release (for maximum optimization) 
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # to fix widget inclusion problem
prepare_qt_binary()

# But still add debug information (-g flag)
#set_target_properties( ${folder_name} PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra" )
