prepare_executable_name()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

set(QT_SOURCES
    main.cpp
    mainwindow.cpp
)

add_executable(${executable_name} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${executable_name}
    Qt5::Core
    Qt5::Widgets
)