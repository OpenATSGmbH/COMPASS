add_library(compass_db "")

include("${CMAKE_CURRENT_LIST_DIR}/command/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/interface/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/dbcontent/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/job/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/util/CMakeLists.txt")

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_sources(compass_db
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/buffer_utils.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/buffer_utils.cpp"
)

target_include_directories(compass_db PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_db
    PUBLIC
        compass_core
        ${SQLITE3_LIBRARIES}
        ${DUCKDB_LIBRARIES}
)
