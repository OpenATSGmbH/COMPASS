
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/correct.h"
        "${CMAKE_CURRENT_LIST_DIR}/false.h"
        "${CMAKE_CURRENT_LIST_DIR}/correct_period.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/correct.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/false.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/correct_period.cpp"
    )
