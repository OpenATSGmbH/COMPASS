add_library(compass_app "")

include("${CMAKE_CURRENT_LIST_DIR}/client/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/watchdog/CMakeLists.txt")

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_sources(compass_app
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/compass.cpp"
)

target_include_directories(compass_app PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_app PUBLIC compass)
