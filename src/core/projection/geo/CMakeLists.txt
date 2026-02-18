
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/geoprojection.h"
        "${CMAKE_CURRENT_LIST_DIR}/geocoordinatesystem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/geoprojection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/geocoordinatesystem.cpp"
)


