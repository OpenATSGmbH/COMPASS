include("${CMAKE_CURRENT_LIST_DIR}/asterix/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/json/CMakeLists.txt")

target_include_directories(compass_task PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
