
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/ogrprojection.h"
        "${CMAKE_CURRENT_LIST_DIR}/ogrcoordinatesystem.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/ogrprojection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ogrcoordinatesystem.cpp"
)


