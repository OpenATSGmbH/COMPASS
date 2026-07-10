
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/adsbdataiteminspector.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsbcoverageinspector.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/adsbdataiteminspector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsbcoverageinspector.cpp"
)
