# Edit the configuration line below to match binary needs
set(binary_dependencies        
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
        Qt5::Network
        Qt5::PrintSupport
)

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

prepare_qt_binary()
