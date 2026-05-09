# DB layer - merged into compass_core

include("${CMAKE_CURRENT_LIST_DIR}/command/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/interface/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/dbcontent/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/job/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/util/CMakeLists.txt")

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/buffer_utils.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/buffer_utils.cpp"
)

target_include_directories(compass_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
