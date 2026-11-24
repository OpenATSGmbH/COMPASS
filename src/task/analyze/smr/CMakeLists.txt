
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/smrdataiteminspector.h"
        "${CMAKE_CURRENT_LIST_DIR}/smrcoverageinspector.h"
        "${CMAKE_CURRENT_LIST_DIR}/smr_coverage_helpers.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/smrdataiteminspector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/smrcoverageinspector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/smr_coverage_helpers.cpp"
)
