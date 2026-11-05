add_library(compass_core "")

include("${CMAKE_CURRENT_LIST_DIR}/base/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/property/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/buffer/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/config/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/unit/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/projection/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/rtcommand/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/net/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/job/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/license/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/sector/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/context/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/source/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/gui/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/latex/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/db/CMakeLists.txt")

target_sources(compass_core
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/compass.h"
    "${CMAKE_CURRENT_LIST_DIR}/appmode.h"
    "${CMAKE_CURRENT_LIST_DIR}/appstate.h"
    "${CMAKE_CURRENT_LIST_DIR}/imainwindow.h"
    "${CMAKE_CURRENT_LIST_DIR}/iviewprovider.h"
    "${CMAKE_CURRENT_LIST_DIR}/ifilterprovider.h"
    "${CMAKE_CURRENT_LIST_DIR}/ievalprovider.h"
    "${CMAKE_CURRENT_LIST_DIR}/viewpointkeys.h"
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/compass.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/appstate.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/viewpointkeys.cpp"
)

target_include_directories(compass_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

if(EXPERIMENTAL_SRC)
    # extra_resources.h lives in experimental_src/util/ but is needed by
    # core's license.cpp. Add the include dir + source to compass_core
    # so the dependency stays downward (core doesn't link experimental libs).
    target_include_directories(compass_core PUBLIC
        "${CMAKE_SOURCE_DIR}/experimental_src/util"
        "${CMAKE_SOURCE_DIR}/experimental_src/test"
    )
    target_sources(compass_core PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/util/extra_resources.h"
        "${CMAKE_SOURCE_DIR}/experimental_src/util/extra_resources.cpp"
    )
endif()

target_link_libraries(compass_core
    PUBLIC
        compass_util
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
        ${DUCKDB_LIBRARIES}
        Eigen3::Eigen
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEARTH_LIBRARY}
)

compass_automoc(compass_core)
