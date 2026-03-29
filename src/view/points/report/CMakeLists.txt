target_include_directories(compass_view PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_view
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsreportgenerator.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsreportgeneratordialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsreportgenerator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewpointsreportgeneratordialog.cpp"
)


