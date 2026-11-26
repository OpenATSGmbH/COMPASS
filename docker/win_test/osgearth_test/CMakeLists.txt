cmake_minimum_required(VERSION 3.16)

project(osgearth_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSceneGraph REQUIRED COMPONENTS osgViewer osgGA osgDB osgUtil OpenThreads)
find_package(osgEarth CONFIG REQUIRED)
find_package(GDAL CONFIG REQUIRED)

# console application, so that stdout is visible in Wine and cmd.exe
add_executable(osgearth_test main.cpp)
target_include_directories(osgearth_test PRIVATE ${OPENSCENEGRAPH_INCLUDE_DIRS})
target_link_libraries(osgearth_test osgEarth::osgEarth GDAL::GDAL ${OPENSCENEGRAPH_LIBRARIES} opengl32)

install(TARGETS osgearth_test RUNTIME DESTINATION .)
install(FILES test.earth DESTINATION .)
