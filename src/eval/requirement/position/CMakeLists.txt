target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/distanceconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/distanceconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/distance.h"
        "${CMAKE_CURRENT_LIST_DIR}/distancerms.h"
        "${CMAKE_CURRENT_LIST_DIR}/distancermsconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/distancermsconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/along.h"
        "${CMAKE_CURRENT_LIST_DIR}/alongconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/alongconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/across.h"
        "${CMAKE_CURRENT_LIST_DIR}/acrossconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/acrossconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/latency.h"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarrange.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarrangeconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarrangeconfigwidget.h"        
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuth.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuthconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuthconfigwidget.h"        
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/distanceconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/distanceconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/distance.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/distancerms.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/distancermsconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/distancermsconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/along.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/alongconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/alongconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/across.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acrossconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acrossconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latency.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latencyconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarrange.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarrangeconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarrangeconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuth.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuthconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarazimuthconfigwidget.cpp"
        
    )


