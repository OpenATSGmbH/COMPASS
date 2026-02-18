include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrack.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrackconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrackconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustarget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustargetconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustargetconfigwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrack.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrackconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustrackconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustarget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustargetconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dubioustargetconfigwidget.cpp"        
    )


