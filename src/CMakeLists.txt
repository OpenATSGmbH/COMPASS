
include("${CMAKE_CURRENT_LIST_DIR}/buffer/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/asterix/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/client/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/config/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/command/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/dbcontent/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/interface/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/util/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/latex/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gui/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/net/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/filter/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/job/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/json/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/projection/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/property/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/sector/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/source/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/task/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/view/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/unit/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/eval/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/rtcommand/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/reconstruction/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/fft/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/license/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/watchdog/CMakeLists.txt")

include("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/compass.h"
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/compass.cpp"
)
