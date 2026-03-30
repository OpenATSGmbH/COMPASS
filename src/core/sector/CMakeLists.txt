
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/sector.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectorlayer.h"
        "${CMAKE_CURRENT_LIST_DIR}/airspace.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/sector.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectorlayer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/airspace.cpp"
)


