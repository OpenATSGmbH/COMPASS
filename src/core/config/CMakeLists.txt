
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/configurable.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurableparameter.h"
        "${CMAKE_CURRENT_LIST_DIR}/configuration.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurationmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/configjson.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/configurable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurableparameter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configuration.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurationmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configjson.cpp"
    )


