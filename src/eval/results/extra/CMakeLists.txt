
target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/data.h"
        "${CMAKE_CURRENT_LIST_DIR}/track.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/data.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/track.cpp"
    )


