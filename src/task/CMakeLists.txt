add_library(compass_task SHARED "")

# COMPASS_TASK_API for the classes with signals (top-level CMakeLists.txt)
generate_export_header(compass_task
    EXPORT_MACRO_NAME COMPASS_TASK_API
    EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/compass_task_export.h")
target_include_directories(compass_task PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/analyze/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/assoc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/calc/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/import/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/manage/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/reconstructor/CMakeLists.txt")

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/taskmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/task.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/taskmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.cpp"
)

target_include_directories(compass_task PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_task
    PUBLIC
        compass_core
        compass_reconstruction
        compass_gui
        compass_filter
        compass_view
        compass_result
        compass_eval
        NemaTode::NemaTode
        ${LibArchive_LIBRARIES}
)

compass_automoc(compass_task)
