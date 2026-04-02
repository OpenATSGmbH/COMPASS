
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dbcommand.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcommandlist.h"
    PRIVATE
)

