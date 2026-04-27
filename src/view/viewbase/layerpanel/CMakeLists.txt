
target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/layertreeitem.h"
        "${CMAKE_CURRENT_LIST_DIR}/layertreemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/layertreeitemdelegate.h"
        "${CMAKE_CURRENT_LIST_DIR}/layerpanelwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentlayer.h"
        "${CMAKE_CURRENT_LIST_DIR}/annotationsrootitem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/layertreeitem.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/layertreemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/layertreeitemdelegate.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/layerpanelwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentlayer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/annotationsrootitem.cpp"
)
