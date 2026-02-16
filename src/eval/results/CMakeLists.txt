
include("${CMAKE_CURRENT_LIST_DIR}/base/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/report/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/detection/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/position/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/speed/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/trackangle/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/identification/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/mode_a/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/mode_c/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/extra/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/dubious/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/generic/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/evaluationresultsgenerator.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdetail.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtaskresult.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/evaluationresultsgenerator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdetail.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtaskresult.cpp"
    )
