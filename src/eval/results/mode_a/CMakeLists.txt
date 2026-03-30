
target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/present.h"
        "${CMAKE_CURRENT_LIST_DIR}/false.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/present.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/false.cpp"
    )


