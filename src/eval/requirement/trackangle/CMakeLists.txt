include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/trackangle.h"
        "${CMAKE_CURRENT_LIST_DIR}/trackangleconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/trackangleconfigwidget.h"
        #"${CMAKE_CURRENT_LIST_DIR}/detail.h"        
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/trackangle.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trackangleconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trackangleconfigwidget.cpp"
    )


