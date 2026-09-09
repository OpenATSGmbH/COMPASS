target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/latency.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatency.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/latency.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatency.cpp"
    )
