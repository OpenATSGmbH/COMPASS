
include("${CMAKE_CURRENT_LIST_DIR}/rs2g/CMakeLists.txt")
#include("${CMAKE_CURRENT_LIST_DIR}/ogr/CMakeLists.txt")
#include("${CMAKE_CURRENT_LIST_DIR}/geo/CMakeLists.txt")

target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/projectionmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/projectionmanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/projection.h"
        "${CMAKE_CURRENT_LIST_DIR}/projectioncoordinatesystembase.h"
        "${CMAKE_CURRENT_LIST_DIR}/transformation.h"
        "${CMAKE_CURRENT_LIST_DIR}/projector.h"
        "${CMAKE_CURRENT_LIST_DIR}/rasterreference.h"
        "${CMAKE_CURRENT_LIST_DIR}/radarbiasinfo.h"
        "${CMAKE_CURRENT_LIST_DIR}/coordconverter.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/projectionmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/projectionmanagerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/projection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/projectioncoordinatesystembase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/transformation.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/projector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/coordconverter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/radarbiasinfo.cpp"
)


