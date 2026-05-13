
include(${CMAKE_CURRENT_LIST_DIR}/label/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/target/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/variable/CMakeLists.txt)

target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentaccessor.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentvariablelookup.h"
        "${CMAKE_CURRENT_LIST_DIR}/bufferaccessor.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/loadrequest.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentstatusinfo.h"
        "${CMAKE_CURRENT_LIST_DIR}/stringrepresentationcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/selectdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentdatastore.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemprovider.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemmodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemview.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_content_edit_dialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentaccessor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentvariablelookup.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/bufferaccessor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanagerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentstatusinfo.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentdatastore.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemprovider.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemmodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentitemview.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_content_edit_dialog.cpp"
)
