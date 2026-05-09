
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/mlatdataiteminspector.h"
        "${CMAKE_CURRENT_LIST_DIR}/mlatcoverageinspector.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/mlatdataiteminspector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mlatcoverageinspector.cpp"
)
