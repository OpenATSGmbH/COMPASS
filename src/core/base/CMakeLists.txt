target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/targetposition.h"
        "${CMAKE_CURRENT_LIST_DIR}/idbvariableresolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/colormap_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/colorlegend.h"
        "${CMAKE_CURRENT_LIST_DIR}/colormap.h"
        "${CMAKE_CURRENT_LIST_DIR}/colorscales.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/colorlegend.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/colormap.cpp"
)
