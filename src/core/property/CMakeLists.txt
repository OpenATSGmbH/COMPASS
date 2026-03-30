
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/property.h"
        "${CMAKE_CURRENT_LIST_DIR}/propertylist.h"
        "${CMAKE_CURRENT_LIST_DIR}/property_templates.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/property.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/propertylist.cpp"
    )
