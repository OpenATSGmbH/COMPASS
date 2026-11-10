add_library(compass_result "")

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

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_result PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/docx"
        "${CMAKE_SOURCE_DIR}/experimental_src/view"
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

compass_automoc(compass_result)
