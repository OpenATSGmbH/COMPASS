add_library(compass_view "")

include("${CMAKE_CURRENT_LIST_DIR}/tableview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/histogramview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/scatterplotview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gridview/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/viewbase/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/points/CMakeLists.txt")

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
        compass_gui
        compass_eval
)

# view sources include headers from report and app - circular deps
target_include_directories(compass_view PRIVATE
    "${CMAKE_SOURCE_DIR}/src/report"
    "${CMAKE_SOURCE_DIR}/src/report/latex"
    "${CMAKE_SOURCE_DIR}/src/report/latex/visitor"
    "${CMAKE_SOURCE_DIR}/src/app/client"
)

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_view PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/view"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/annotations"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/drawable"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/ffts"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/geometry"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/map"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/measurement"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/overlay"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/radars"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/remoteunits"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/sector"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/style"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/style/rule"
    )
endif()

compass_automoc(compass_view)
