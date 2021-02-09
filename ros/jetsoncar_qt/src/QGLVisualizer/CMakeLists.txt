cmake_minimum_required(VERSION 3.5)

# Configure build as release (for maximum optimization) but still add debug information (-g flag)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

# Name the library in this folder as the folder name
get_filename_component(folder_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" folder_name ${folder_name})
set(library_name "${folder_name}")

message(STATUS "   ${library_name}\t\t[${CMAKE_BUILD_TYPE}]")

# Make a compiler flag for the current source folder path
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__SOURCE_FOLDER__='\"${CMAKE_CURRENT_SOURCE_DIR}\"'")

set(library_dependencies 
  ${catkin_LIBRARIES} 
  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Xml Qt5::PrintSupport
  Qt5::OpenGL
  Qt5::3DCore Qt5::3DRender Qt5::3DInput Qt5::3DExtras 
  ${QGLVIEWER_LIBRARY}  
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenGL_LIBRARIES}
  ${GLUT_LIBRARIES}  
)

FILE(GLOB_RECURSE binary_sources_c *.c) # find all c files
FILE(GLOB_RECURSE binary_sources_cpp *.cpp) # find all cpp files
FILE(GLOB_RECURSE binary_headers *.hpp) # find all hpp files
FILE(GLOB_RECURSE general_headers *.h) # find all h files

#set(GLOBAL_HEADERS ${GLOBAL_HEADERS} ${binary_headers} ${general_headers} PARENT_SCOPE)
set_property(SOURCE ${binary_sources_c} PROPERTY SKIP_AUTOGEN ON)
set_property(SOURCE ${binary_sources_cpp} PROPERTY SKIP_AUTOGEN ON)
set_property(SOURCE ${GLOBAL_HEADERS} PROPERTY SKIP_AUTOGEN ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)

# Add the library - change SHARED/STATIC according to the requirement
add_library( ${library_name} SHARED ${binary_sources_c} ${binary_sources_cpp} ${GLOBAL_HEADERS} )
add_dependencies( ${library_name} ${catkin_EXPORTED_TARGETS})
target_link_libraries( ${library_name} ${library_dependencies} )
target_include_directories( ${library_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

set_target_properties( ${library_name} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra" )
