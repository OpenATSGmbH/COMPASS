
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dubiousbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrack.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustarget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dubiousbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrack.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustarget.cpp"
    )


