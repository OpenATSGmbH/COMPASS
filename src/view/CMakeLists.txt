add_library(compass_view SHARED "")

# COMPASS_VIEW_API for the classes with signals (top-level CMakeLists.txt)
generate_export_header(compass_view
    EXPORT_MACRO_NAME COMPASS_VIEW_API
    EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/compass_view_export.h")
target_include_directories(compass_view PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/tableview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/histogramview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/scatterplotview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gridview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/viewbase/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/points/CMakeLists.txt")

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/viewmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/loadcontroller.h"
        "${CMAKE_CURRENT_LIST_DIR}/livecontroller.h"
        "${CMAKE_CURRENT_LIST_DIR}/liveoverloaddetector.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewmanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainer.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewdeck.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportfigure.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportfigureoptions.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportfiguresettingswidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/viewmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/loadcontroller.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/livecontroller.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/liveoverloaddetector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewmanagerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewdeck.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportfigure.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportfigureoptions.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportfiguresettingswidget.cpp"
)

target_include_directories(compass_view PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_view
    PUBLIC
        compass_core
        compass_gui
        compass_filter
)

compass_automoc(compass_view)
