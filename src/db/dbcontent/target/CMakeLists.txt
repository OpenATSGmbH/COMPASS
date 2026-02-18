
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_db
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/targetreportdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/target.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetadsbinfo.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetmodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetlistwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetreportchain.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetposition.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetpositionaccuracy.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetvelocity.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetreportaccessor.h"
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/targetreportdefs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/target.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetadsbinfo.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetmodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetlistwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetreportchain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetpositionaccuracy.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetvelocity.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetreportaccessor.cpp"
        )


