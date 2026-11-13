
target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/grid2d_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/grid2d.h"
        "${CMAKE_CURRENT_LIST_DIR}/grid2dlayer.h"
        "${CMAKE_CURRENT_LIST_DIR}/grid2dlayerrenderer.h"
        "${CMAKE_CURRENT_LIST_DIR}/grid2drendersettings.h"

        "${CMAKE_CURRENT_LIST_DIR}/gridview.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewdatawidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewchart.h"

        "${CMAKE_CURRENT_LIST_DIR}/gridbackground_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgroundtilesource.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgroundrenderer.h"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgrounditems.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/grid2d_defs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/grid2d.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/grid2dlayer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/grid2dlayerrenderer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/grid2drendersettings.cpp"

        "${CMAKE_CURRENT_LIST_DIR}/gridview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewdatawidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridviewchart.cpp"

        "${CMAKE_CURRENT_LIST_DIR}/gridbackground_defs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgroundtilesource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgroundrenderer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gridbackgrounditems.cpp"
)
