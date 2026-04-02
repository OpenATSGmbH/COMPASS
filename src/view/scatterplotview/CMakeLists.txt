
target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewdatasource.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotview.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewchartview.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewdatawidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseries.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseriesmodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseriestreeitem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewdatasource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewchartview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewdatawidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterplotviewwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseries.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseriesmodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scatterseriestreeitem.cpp"
)
