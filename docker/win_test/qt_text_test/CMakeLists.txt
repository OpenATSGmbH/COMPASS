cmake_minimum_required(VERSION 3.16)

project(qt_text_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(/opt/qt5_host_tools.cmake)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(qt_text_test main.cpp)
target_link_libraries(qt_text_test Qt5::Widgets Qt5::Gui Qt5::Core)

set(QT_BIN_DIR $ENV{QT_WIN_DIR}/bin)
set(QT_PLUGIN_DIR $ENV{QT_WIN_DIR}/plugins)

install(TARGETS qt_text_test RUNTIME DESTINATION .)
install(FILES ${QT_BIN_DIR}/Qt5Core.dll ${QT_BIN_DIR}/Qt5Gui.dll ${QT_BIN_DIR}/Qt5Widgets.dll DESTINATION .)
install(FILES ${QT_PLUGIN_DIR}/platforms/qwindows.dll DESTINATION platforms)
