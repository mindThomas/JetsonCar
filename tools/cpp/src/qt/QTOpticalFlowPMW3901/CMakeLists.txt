# Edit the configuration line below to match binary needs
set(binary_dependencies
        ${Eigen3_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Qt5::Core
        Qt5::Charts
        Qt5::Widgets
        Qt5::Network
        Qt5::PrintSupport
        Qt5::SerialPort
)

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

prepare_qt_binary()
