
include("${CMAKE_CURRENT_LIST_DIR}/report/CMakeLists.txt")

target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.cpp"
    )
