
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/sector.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectorlayer.h"
        "${CMAKE_CURRENT_LIST_DIR}/airspace.h"
        "${CMAKE_CURRENT_LIST_DIR}/sector_edit_widget.h"
        "${CMAKE_CURRENT_LIST_DIR}/sector_import_utils.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/sector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectorlayer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/airspace.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sector_edit_widget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sector_import_utils.cpp"
)


