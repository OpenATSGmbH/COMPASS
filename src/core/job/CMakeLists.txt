
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/job.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffercsvexportjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/buffercsvexportjob.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/allbuffercsvexportjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/buffercsvexportjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jobmanager.cpp"
)
