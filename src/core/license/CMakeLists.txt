
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/license.h"
        "${CMAKE_CURRENT_LIST_DIR}/licensemanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/licensewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/licenseimportdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/licensemanagerdialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/license.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/licensemanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/licensewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/licenseimportdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/licensemanagerdialog.cpp"
)
