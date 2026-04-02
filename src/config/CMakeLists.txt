add_library(compass_config "")

target_sources(compass_config
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/config_paths.h"
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

target_include_directories(compass_config PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    # traced_assert.h lives in core/util but is needed by config headers.
    # Cannot link compass_core (circular dep), so expose the dir directly.
    "${CMAKE_CURRENT_LIST_DIR}/../core/util"
)

target_link_libraries(compass_config
    PUBLIC
        ${Boost_LIBRARIES}
        ${LOG4CPP_LIBRARIES}
        stdc++fs # GCC 8 (Debian 10) needs this for std::filesystem; harmless no-op on GCC 9+
)
