# Edit the configuration line below to match binary needs
set(binary_dependencies   ${Boost_LIBRARIES} ${PCL_LIBRARIES} )

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "Release")

prepare_qt_binary()

# But still add debug information (-g flag)
#set_target_properties( ${folder_name} PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra" )
