target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttask.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttaskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttaskdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonparsejob.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonmappingjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonmappingstubsjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/readjsonfilejob.h"        
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttaskwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonimporttaskdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonparsejob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonmappingjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonmappingstubsjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/readjsonfilejob.cpp"        
)


