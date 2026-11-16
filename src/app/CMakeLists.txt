add_library(compass_app SHARED "")

# Disable AUTOMOC: compass_app links libraries (compass_view, compass_eval, etc.)
# whose PUBLIC include dirs expose Q_OBJECT headers. AUTOMOC would re-MOC those
# headers, creating duplicate symbols with the library that already MOC'd them.
# Instead, manually MOC only compass_app's own Q_OBJECT headers.
set_target_properties(compass_app PROPERTIES AUTOMOC OFF)
qt5_wrap_cpp(COMPASS_APP_MOC_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/compass.h"
    "${CMAKE_CURRENT_LIST_DIR}/client/mainwindow.h"
    "${CMAKE_CURRENT_LIST_DIR}/client/toolbox.h"
)
target_sources(compass_app PRIVATE ${COMPASS_APP_MOC_SRCS})

include("${CMAKE_CURRENT_LIST_DIR}/client/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/watchdog/CMakeLists.txt")

target_sources(compass_app
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/apphooks.h"
        "${CMAKE_CURRENT_LIST_DIR}/compass.h"
        "${CMAKE_CURRENT_LIST_DIR}/compassroot.h"
        "${CMAKE_CURRENT_LIST_DIR}/plugin/plugin.h"
        "${CMAKE_CURRENT_LIST_DIR}/plugin/pluginmanager.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/compass.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/compassroot.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/plugin/pluginmanager.cpp"
)

target_include_directories(compass_app PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_app
    PRIVATE
        compass_config
        compass_core
        compass_reconstruction
        compass_gui
        compass_filter
        compass_task
        compass_eval
        compass_view
        compass_result
        Qt5::Widgets
        Qt5::Core
        Qt5::OpenGL
        Qt5::Charts
        Qt5::Test
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEARTH_LIBRARY}
)

# No experimental libraries are linked here, only by compass_client: they extend the
# application through AppHooks (incl. the plugin objects), set in compass_client's main.cpp.

# compass_plugin_sdk: the host libraries a plugin may link (readme_plugins.md). Plugins link this
# target instead of the libraries, so the set can change (e.g. a core/UI split) without touching
# the plugins' CMake files. compass_app: the Plugin interface (plugin/plugin.h)
add_library(compass_plugin_sdk INTERFACE)
target_link_libraries(compass_plugin_sdk INTERFACE
    compass_core
    compass_reconstruction
    compass_gui
    compass_view
    compass_result
    compass_task
    compass_app
)
