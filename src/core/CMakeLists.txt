add_library(compass_core "")

include("${CMAKE_CURRENT_LIST_DIR}/property/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/buffer/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/config/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/util/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/unit/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/command/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/projection/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/rtcommand/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/net/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/job/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/latex/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/license/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/sector/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/source/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_sources(compass_core
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/compass.h"
    "${CMAKE_CURRENT_LIST_DIR}/appmode.h"
)

target_include_directories(compass_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_core
    PUBLIC
        compass_config
        Qt5::Widgets
        Qt5::Core
        Qt5::OpenGL
        ${Boost_LIBRARIES}
        ${LOG4CPP_LIBRARIES}
        ${GDAL_LIBRARIES}
        ${GeographicLib_LIBRARIES}
        ${PCAP_LIBRARY}
        ${OPENSSL_CRYPTO_LIBRARY}
        ${TBB_LIBRARIES}
        TBB::tbb
        Threads::Threads
        ${CMAKE_DL_LIBS}
)
