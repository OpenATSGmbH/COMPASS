
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
#        "${CMAKE_CURRENT_LIST_DIR}/managedbcontenttask.h"
#        "${CMAKE_CURRENT_LIST_DIR}/managedbcontenttaskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstask.h"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstaskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstaskdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/importsectordialog.h"
    PRIVATE
#        "${CMAKE_CURRENT_LIST_DIR}/managedbcontenttask.cpp"
#        "${CMAKE_CURRENT_LIST_DIR}/managedbcontenttaskwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstaskwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/managesectorstaskdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/importsectordialog.cpp"
    )
