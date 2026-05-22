target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttask.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttaskdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttaskwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttaskdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsimporttaskwidget.cpp"
)


