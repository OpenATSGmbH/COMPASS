target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/latency.h"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatency.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatencyconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatencyconfigwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/latency.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatency.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatencyconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsblatencyconfigwidget.cpp"
    )
