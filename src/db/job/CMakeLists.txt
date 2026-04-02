target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentreaddbjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentdeletedbjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentinsertdbjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/updatebufferdbjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffercsvexportjob.h"
        "${CMAKE_CURRENT_LIST_DIR}/buffercsvexportjob.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentreaddbjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentdeletedbjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbcontentinsertdbjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/updatebufferdbjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/allbuffercsvexportjob.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/buffercsvexportjob.cpp"
)
