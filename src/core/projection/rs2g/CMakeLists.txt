
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/rs2gprojection.h"
        "${CMAKE_CURRENT_LIST_DIR}/rs2gcoordinatesystem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/rs2gprojection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rs2gcoordinatesystem.cpp"
)


