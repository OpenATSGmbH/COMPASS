cmake_minimum_required(VERSION 3.16)

project(qt_startup_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(/opt/qt5_host_tools.cmake)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(qt_startup_test main.cpp)
target_link_libraries(qt_startup_test Qt5::Widgets Qt5::Gui Qt5::Core)

install(TARGETS qt_startup_test RUNTIME DESTINATION .)
