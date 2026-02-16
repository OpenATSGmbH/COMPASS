include("${CMAKE_CURRENT_LIST_DIR}/report/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/viewabledataconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpoint.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpoint_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointstoolwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointstablemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointgenerator.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/viewpoint.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpoint_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointstoolwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointstablemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointgenerator.cpp"
)
