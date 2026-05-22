target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstask.h"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstaskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationsstatusdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstaskdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationsjob.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstaskwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationsstatusdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationstaskdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/createartasassociationsjob.cpp"      
)
