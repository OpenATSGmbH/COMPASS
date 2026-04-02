
include("${CMAKE_CURRENT_LIST_DIR}/asterix/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gps/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/json/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/view_points/CMakeLists.txt")

target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )
