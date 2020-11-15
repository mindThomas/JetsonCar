# Edit the configuration line below to match binary needs
set(binary_dependencies        
        ${Boost_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
        Qt5::Network
        Qt5::PrintSupport
) # ${OpenCV_LIBRARIES}

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "Release")

prepare_qt_binary()