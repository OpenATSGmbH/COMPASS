add_library(compass_task "")

include("${CMAKE_CURRENT_LIST_DIR}/assoc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/calc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/import/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/manage/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/reconstructor/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/result/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

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
        compass_db
        compass_reconstruction
        compass_import
        compass_gui
        compass_filter
        NemaTode::NemaTode
        ${LibArchive_LIBRARIES}
)
