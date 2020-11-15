cmake_minimum_required(VERSION 3.5)

# Configure build as release (for maximum optimization) but still add debug information (-g flag)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

# Name the binary in this folder as the folder name
get_filename_component(folder_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" folder_name ${folder_name})
set(library_name "${folder_name}")

message(STATUS "   ${library_name}\t\t[${CMAKE_BUILD_TYPE}]")

# Make a compiler flag for the current source folder path
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__SOURCE_FOLDER__='\"${CMAKE_CURRENT_SOURCE_DIR}\"'")

set(library_dependencies        
        ${OpenCV_LIBRARIES}
        agast
    )

FILE(GLOB_RECURSE binary_sources_c *.c) # find all c files
FILE(GLOB_RECURSE binary_sources_cpp *.cc) # find all cpp files
FILE(GLOB_RECURSE binary_headers *.hpp) # find all hpp files
FILE(GLOB_RECURSE general_headers *.h) # find all h files

#set(GLOBAL_HEADERS ${GLOBAL_HEADERS} ${binary_headers} ${general_headers} PARENT_SCOPE)
set_property(SOURCE ${binary_sources_c} PROPERTY SKIP_AUTOGEN ON)
set_property(SOURCE ${binary_sources_cpp} PROPERTY SKIP_AUTOGEN ON)
set_property(SOURCE ${GLOBAL_HEADERS} PROPERTY SKIP_AUTOGEN ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)

# Add the library - change SHARED/STATIC according to the requirement
add_library( ${library_name} STATIC ${binary_sources_c} ${binary_sources_cpp} ${GLOBAL_HEADERS} )
target_link_libraries( ${library_name} ${library_dependencies} )
target_include_directories( ${library_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64*")
  message(STATUS "ARM 64bit processor detected, will use NEON by default.")
  set_target_properties( ${library_name} PROPERTIES COMPILE_FLAGS "-std=c++0x -fPIC -Wall -Werror -pedantic")
  target_compile_options( ${library_name} PUBLIC "-D__ARM_NEON__ -DHAVE_OPENCV")
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm.*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")  
  set_target_properties( ${library_name} PROPERTIES COMPILE_FLAGS "-mfpu=neon -Wall -Werror -pedantic -std=c++0x -fPIC")
  target_compile_options( ${library_name} PUBLIC "-DHAVE_OPENCV")  
else()
  message(STATUS "Assuming SSE instructions available.")  
  set_target_properties( ${library_name} PROPERTIES COMPILE_FLAGS "-mssse3 -Wall -Werror -pedantic -std=c++0x -fPIC")
  target_compile_options( ${library_name} PUBLIC "-DHAVE_OPENCV")    
endif()