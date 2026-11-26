add_library(compass_result SHARED "")

# COMPASS_RESULT_API for the classes with signals (top-level CMakeLists.txt)
generate_export_header(compass_result
    EXPORT_MACRO_NAME COMPASS_RESULT_API
    EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/compass_result_export.h")
target_include_directories(compass_result PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/report/CMakeLists.txt")

target_include_directories(compass_result PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_result
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/itaskprovider.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultdb.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultdb.cpp"
    )

target_link_libraries(compass_result
    PUBLIC
        compass_core
        compass_gui
        compass_view
        Qt5::Widgets
        Qt5::Core
)

compass_automoc(compass_result)
