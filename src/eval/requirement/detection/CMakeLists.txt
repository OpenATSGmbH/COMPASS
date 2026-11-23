target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/detectionconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/detectionconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/detection.h"
        "${CMAKE_CURRENT_LIST_DIR}/detection_walk.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/detectionconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/detectionconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/detection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/detection_walk.cpp"
)


