
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/job.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/async.h"
        "${CMAKE_CURRENT_LIST_DIR}/asynctask.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/async.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asynctask.cpp"
)
