add_library(compass_view "")

include("${CMAKE_CURRENT_LIST_DIR}/tableview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/histogramview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/scatterplotview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gridview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/viewbase/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/points/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/viewmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewmanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainer.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainerwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/viewmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewmanagerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewcontainerwidget.cpp"
)

target_include_directories(compass_view PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_view
    PUBLIC
        compass_core
        compass_db
        compass_gui
        compass_eval
)
