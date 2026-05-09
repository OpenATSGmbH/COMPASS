add_library(compass_task "")

include("${CMAKE_SOURCE_DIR}/src/import/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/assoc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/calc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/import/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/manage/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/reconstructor/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/result/CMakeLists.txt")

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/taskdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/task.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/asynctask.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/taskmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asynctask.cpp"
)

target_include_directories(compass_task PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_task
    PUBLIC
        compass_core
        compass_reconstruction
        compass_gui
        compass_filter
        NemaTode::NemaTode
        ${LibArchive_LIBRARIES}
        ${jASTERIX_LIBRARIES}
)

# task sources include headers from app, report, view, eval - circular deps
target_include_directories(compass_task PRIVATE
    "${CMAKE_SOURCE_DIR}/src/app/client"
    "${CMAKE_SOURCE_DIR}/src/report"
    "${CMAKE_SOURCE_DIR}/src/report/latex"
    "${CMAKE_SOURCE_DIR}/src/report/latex/visitor"
    "${CMAKE_SOURCE_DIR}/src/view"
    "${CMAKE_SOURCE_DIR}/src/view/viewbase"
    "${CMAKE_SOURCE_DIR}/src/view/points"
    "${CMAKE_SOURCE_DIR}/src/view/points/report"
    "${CMAKE_SOURCE_DIR}/src/view/histogramview"
    "${CMAKE_SOURCE_DIR}/src/view/scatterplotview"
    "${CMAKE_SOURCE_DIR}/src/view/gridview"
    "${CMAKE_SOURCE_DIR}/src/view/tableview"
    "${CMAKE_SOURCE_DIR}/src/eval"
    "${CMAKE_SOURCE_DIR}/src/eval/data"
    "${CMAKE_SOURCE_DIR}/src/eval/results"
)

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_task PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/docx"
        "${CMAKE_SOURCE_DIR}/experimental_src/libs/osg"
        "${CMAKE_SOURCE_DIR}/experimental_src/reconstruction/complex"
        "${CMAKE_SOURCE_DIR}/experimental_src/reconstruction/jpda"
        "${CMAKE_SOURCE_DIR}/experimental_src/reconstruction/kalman"
        "${CMAKE_SOURCE_DIR}/experimental_src/test"
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

compass_automoc(compass_task)
