
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/job.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.cpp"
)
