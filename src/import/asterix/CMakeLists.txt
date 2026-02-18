
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_import
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/asterixconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixoverridewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixframingcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixeditioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixrefeditioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixspfeditioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixcategoryconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparser.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparserwidget.h"        
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparserdetailwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparsingschema.h"        
        "${CMAKE_CURRENT_LIST_DIR}/asterixpostprocess.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/asterixconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixoverridewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparser.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparserwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparserdetailwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixjsonparsingschema.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterixpostprocess.cpp"
)

target_sources(compass_import
    PUBLIC
    PRIVATE
    )
