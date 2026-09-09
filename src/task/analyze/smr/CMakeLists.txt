
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/smrdataiteminspector.h"
        "${CMAKE_CURRENT_LIST_DIR}/smrcoverageinspector.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/smrdataiteminspector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/smrcoverageinspector.cpp"
)
