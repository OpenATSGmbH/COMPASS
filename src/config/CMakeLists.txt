add_library(compass_config SHARED "")

# COMPASS_CONFIG_API for the classes with signals (top-level CMakeLists.txt)
generate_export_header(compass_config
    EXPORT_MACRO_NAME COMPASS_CONFIG_API
    EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/compass_config_export.h")
target_include_directories(compass_config PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

target_sources(compass_config
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/configurable.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurableparameter.h"
        "${CMAKE_CURRENT_LIST_DIR}/configuration.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurationmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/configjson.h"
        "${CMAKE_CURRENT_LIST_DIR}/subconfigurablefactory.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/configurable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurableparameter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configuration.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurationmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configjson.cpp"
)

target_include_directories(compass_config PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(compass_config
    PUBLIC
        compass_util
        ${Boost_LIBRARIES}
        ${LOG4CPP_LIBRARIES}
        stdc++fs # GCC 8 (Debian 10) needs this for std::filesystem; harmless no-op on GCC 9+
)
