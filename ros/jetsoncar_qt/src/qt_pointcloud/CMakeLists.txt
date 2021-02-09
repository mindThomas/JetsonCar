prepare_executable_name()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

set(QT_SOURCES
    main.cpp
    mainwindow.cpp
)

# Configure build as release (for maximum optimization) 
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_WITH_EASY_PROFILER")

add_executable(${executable_name} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(${executable_name} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${executable_name} ${catkin_LIBRARIES}
    Qt5::Widgets  
    easy_profiler
    QGLVisualizer
    ${PCL_LIBRARIES}    
)
