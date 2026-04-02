add_library(compass_report "")

include("${CMAKE_CURRENT_LIST_DIR}/latex/CMakeLists.txt")

target_include_directories(compass_report PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_report
    PUBLIC
        compass_core
        compass_gui
        compass_task
        compass_eval
        compass_view
        Qt5::Widgets
        Qt5::Core
)

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_report PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/geometry"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/style"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/style/rule"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/overlay"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/map"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/drawable"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/annotations"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/ffts"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/measurement"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/radars"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/remoteunits"
        "${CMAKE_SOURCE_DIR}/experimental_src/view/geographicview/sector"
        "${CMAKE_SOURCE_DIR}/experimental_src/libs/osg"
    )
endif()

compass_automoc(compass_report)
