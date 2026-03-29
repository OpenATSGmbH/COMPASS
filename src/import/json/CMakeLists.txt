
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/jsonparsingschema.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsondatamapping.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsondatamappingwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonobjectparser.h"
        "${CMAKE_CURRENT_LIST_DIR}/jsonobjectparserwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/jsonparsingschema.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsondatamapping.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsondatamappingwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonobjectparser.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/jsonobjectparserwidget.cpp"
)


