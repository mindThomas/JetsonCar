# Dependencies/prerequisites:
#   sudo apt-get install libi2c-dev i2c-tools

# Edit the configuration line below to match binary needs
set(binary_dependencies    m i2c_devices)

# Configure build as release (for maximum optimization) 
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

prepare_binary()

# But still add debug information (-g flag)
#set_target_properties( ${folder_name} PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra" )
