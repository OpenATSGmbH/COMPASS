
include(${CMAKE_CURRENT_LIST_DIR}/label/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/target/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/variable/CMakeLists.txt)

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentaccessor.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentvariablelookup.h"
        "${CMAKE_CURRENT_LIST_DIR}/bufferaccessor.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentmanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentstatusinfo.h"
        "${CMAKE_CURRENT_LIST_DIR}/stringrepresentationcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/selectdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontent_commands.h"
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
)
