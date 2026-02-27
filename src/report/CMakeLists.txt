add_library(compass_report "")

include("${CMAKE_CURRENT_LIST_DIR}/latex/CMakeLists.txt")

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_include_directories(compass_report PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_report
    PUBLIC
        compass_core
        compass_db
        compass_gui
        compass_task
        compass_eval
        compass_view
        Qt5::Widgets
        Qt5::Core
)
