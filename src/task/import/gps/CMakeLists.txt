include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttask.h"
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttaskwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttaskdialog.h"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtask.h"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtaskwidget.h"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtaskdialog.h"        
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttaskwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gpstrailimporttaskdialog.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtask.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtaskwidget.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/gpsimportcsvtaskdialog.cpp"                
)

