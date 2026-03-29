include("${CMAKE_CURRENT_LIST_DIR}/base/CMakeLists.txt")
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
include("${CMAKE_CURRENT_LIST_DIR}/mom/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/status/CMakeLists.txt")

target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/group.h"
#        "${CMAKE_CURRENT_LIST_DIR}/checkdetail.h"
#        "${CMAKE_CURRENT_LIST_DIR}/correctnessdetail.h"
#        "${CMAKE_CURRENT_LIST_DIR}/presentdetail.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/group.cpp"
)


