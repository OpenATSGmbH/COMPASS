
target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewdatasource.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramview.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewdatawidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewchartview.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogram.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogram_raw.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramgenerator.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramgeneratorbuffer.h"
        "${CMAKE_CURRENT_LIST_DIR}/histograminitializer.h"
        "${CMAKE_CURRENT_LIST_DIR}/histogramleafpayload.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/histogramview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewdatasource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewdatawidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramviewchartview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogram_raw.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramgenerator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/histogramgeneratorbuffer.cpp"
)
