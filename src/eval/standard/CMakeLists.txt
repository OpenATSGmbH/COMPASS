
target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandard.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtreemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtreeitem.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardrootitem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandard.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardcombobox.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtreemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtreeitem.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardrootitem.cpp"
    )


