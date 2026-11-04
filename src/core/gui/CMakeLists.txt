
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/textfielddoublevalidator.h"
        "${CMAKE_CURRENT_LIST_DIR}/textfieldhexvalidator.h"
        "${CMAKE_CURRENT_LIST_DIR}/textfieldoctvalidator.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeedit.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider_p.h"
        "${CMAKE_CURRENT_LIST_DIR}/popupmenu.h"
        "${CMAKE_CURRENT_LIST_DIR}/questiondialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/dialogs.h"
        "${CMAKE_CURRENT_LIST_DIR}/waitspinner.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/rangeedit.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/popupmenu.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/questiondialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dialogs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/waitspinner.cpp"
)
