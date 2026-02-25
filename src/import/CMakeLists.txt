add_library(compass_import "")

include("${CMAKE_CURRENT_LIST_DIR}/asterix/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/json/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/fft/CMakeLists.txt")

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_include_directories(compass_import PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_import
    PUBLIC
        compass_core
        compass_db
        ${jASTERIX_LIBRARIES}
)
