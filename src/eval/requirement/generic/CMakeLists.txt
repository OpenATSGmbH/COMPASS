include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/genericconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/genericconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/generic.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/genericconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/genericconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/generic.cpp"
)


