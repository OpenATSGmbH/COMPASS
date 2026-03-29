
target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/generic.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/generic.cpp"
    )


