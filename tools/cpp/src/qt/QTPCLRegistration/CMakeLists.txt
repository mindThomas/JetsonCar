# Edit the configuration line below to match binary needs
set(binary_dependencies
        ${Eigen3_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
)

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "Release")

prepare_qt_binary() 