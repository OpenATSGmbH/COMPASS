
include("${CMAKE_CURRENT_LIST_DIR}/report/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.h"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.h"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgenerator.h"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgeneratordialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/taskresult.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/taskresultswidget.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgenerator.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgeneratordialog.cpp"
    )
