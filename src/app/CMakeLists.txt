add_library(compass_app "")

# Disable AUTOMOC: compass_app links libraries (compass_view, compass_eval, etc.)
# whose PUBLIC include dirs expose Q_OBJECT headers. AUTOMOC would re-MOC those
# headers, creating duplicate symbols with the library that already MOC'd them.
# Instead, manually MOC only compass_app's own Q_OBJECT headers.
set_target_properties(compass_app PROPERTIES AUTOMOC OFF)
qt5_wrap_cpp(COMPASS_APP_MOC_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/client/mainwindow.h"
    "${CMAKE_CURRENT_LIST_DIR}/client/toolbox.h"
)
target_sources(compass_app PRIVATE ${COMPASS_APP_MOC_SRCS})

include("${CMAKE_CURRENT_LIST_DIR}/client/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/watchdog/CMakeLists.txt")

target_sources(compass_app
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/compass.cpp"
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
        compass_report
        Qt5::Widgets
        Qt5::Core
        Qt5::OpenGL
        Qt5::Charts
        Qt5::Test
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEARTH_LIBRARY}
)

if(EXPERIMENTAL_SRC)
    # No experimental libraries are linked here — only by compass_client.
    # Linking them would propagate their PUBLIC include dirs into compass_app,
    # causing AUTOMOC to discover Q_OBJECT headers and generate duplicate moc
    # symbols (already defined in the component libraries).
    # Only the geo_view_api.h facade header is needed (no Q_OBJECT).
    target_include_directories(compass_app PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../../experimental_src/view"
    )
endif()
