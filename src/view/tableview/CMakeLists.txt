
target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/basetablemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/basetablewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffertablemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffertablewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableleafpayload.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewdatasource.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableview.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewdatawidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/basetablemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/basetablewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffertablemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffertablewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tableview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewdatasource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewdatawidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tableviewwidget.cpp"
)
