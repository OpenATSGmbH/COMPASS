cmake_minimum_required(VERSION 3.16)

project(qt_osgearth_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Geographic View sources (experimental_src/view/geographicview), compiled unchanged
set(GEOVIEW_DIR "" CACHE PATH "experimental_src/view/geographicview")
if(NOT EXISTS "${GEOVIEW_DIR}/ViewerWidget.cpp")
    message(FATAL_ERROR "GEOVIEW_DIR must point to experimental_src/view/geographicview")
endif()

include(/opt/qt5_host_tools.cmake)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(OpenSceneGraph REQUIRED COMPONENTS osgViewer osgGA osgDB osgUtil osgText OpenThreads)
find_package(osgEarth CONFIG REQUIRED)

# console application, so that stdout is visible in Wine and cmd.exe
add_executable(qt_osgearth_test
    main.cpp
    ${GEOVIEW_DIR}/GraphicsWindowQt.h
    ${GEOVIEW_DIR}/GraphicsWindowQt.cpp
    ${GEOVIEW_DIR}/ViewerWidget.h
    ${GEOVIEW_DIR}/ViewerWidget.cpp)

# shim first: test replacements for the COMPASS logger.h, stringconv.h, compass.h
target_include_directories(qt_osgearth_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/shim ${GEOVIEW_DIR} ${OPENSCENEGRAPH_INCLUDE_DIRS})

# Geographic View classes are compiled into the executable, no DLL import/export
target_compile_definitions(qt_osgearth_test PRIVATE OSGEARTHQT_LIBRARY_STATIC)

target_link_libraries(qt_osgearth_test
    Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL
    osgEarth::osgEarth ${OPENSCENEGRAPH_LIBRARIES} opengl32)

set(QT_BIN_DIR $ENV{QT_WIN_DIR}/bin)
set(QT_PLUGIN_DIR $ENV{QT_WIN_DIR}/plugins)

install(TARGETS qt_osgearth_test RUNTIME DESTINATION .)
install(FILES
    ${QT_BIN_DIR}/Qt5Core.dll
    ${QT_BIN_DIR}/Qt5Gui.dll
    ${QT_BIN_DIR}/Qt5Widgets.dll
    ${QT_BIN_DIR}/Qt5OpenGL.dll
    DESTINATION .)
install(FILES ${QT_PLUGIN_DIR}/platforms/qwindows.dll DESTINATION platforms)
