cmake_minimum_required(VERSION 3.16)

project(opengl_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# console application, so that stdout is visible in Wine and cmd.exe
add_executable(opengl_test main.cpp)
target_link_libraries(opengl_test Qt5::Widgets Qt5::Gui Qt5::Core opengl32)

# deploy runtime DLLs and the Qt platform plugin next to the executable
set(QT_BIN_DIR $ENV{QT_WIN_DIR}/bin)
set(QT_PLUGIN_DIR $ENV{QT_WIN_DIR}/plugins)

install(TARGETS opengl_test RUNTIME DESTINATION .)
install(FILES
    ${QT_BIN_DIR}/Qt5Core.dll
    ${QT_BIN_DIR}/Qt5Gui.dll
    ${QT_BIN_DIR}/Qt5Widgets.dll
    $ENV{MINGW_GCC_DLL_DIR}/libstdc++-6.dll
    $ENV{MINGW_GCC_DLL_DIR}/libgcc_s_seh-1.dll
    $ENV{MINGW_SYS_DLL_DIR}/libwinpthread-1.dll
    DESTINATION .)
install(FILES ${QT_PLUGIN_DIR}/platforms/qwindows.dll DESTINATION platforms)
