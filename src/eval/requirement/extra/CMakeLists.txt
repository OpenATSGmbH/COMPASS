include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dataconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/dataconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/data.h"
        "${CMAKE_CURRENT_LIST_DIR}/trackconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/trackconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/track.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dataconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dataconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/data.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trackconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trackconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/track.cpp"
    )


