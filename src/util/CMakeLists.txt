add_library(compass_util SHARED "")

target_sources(compass_util
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/config.h"
        "${CMAKE_CURRENT_LIST_DIR}/config_paths.h"
        "${CMAKE_CURRENT_LIST_DIR}/files.h"
        # "${CMAKE_CURRENT_LIST_DIR}/global.h"
        "${CMAKE_CURRENT_LIST_DIR}/number.h"
        "${CMAKE_CURRENT_LIST_DIR}/stringconv.h"
        "${CMAKE_CURRENT_LIST_DIR}/json_tools.h"
        "${CMAKE_CURRENT_LIST_DIR}/singleton.h"
        "${CMAKE_CURRENT_LIST_DIR}/system.h"
        "${CMAKE_CURRENT_LIST_DIR}/logger.h"
        "${CMAKE_CURRENT_LIST_DIR}/format.h"
        "${CMAKE_CURRENT_LIST_DIR}/tbbhack.h"
        "${CMAKE_CURRENT_LIST_DIR}/timeconv.h"
        "${CMAKE_CURRENT_LIST_DIR}/event_log.h"
        "${CMAKE_CURRENT_LIST_DIR}/stringmat.h"
        "${CMAKE_CURRENT_LIST_DIR}/result.h"
        "${CMAKE_CURRENT_LIST_DIR}/logstream.h"
        "${CMAKE_CURRENT_LIST_DIR}/logmodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/timewindow.h"
        "${CMAKE_CURRENT_LIST_DIR}/timeddataseries.h"
        "${CMAKE_CURRENT_LIST_DIR}/msghandler.h"
        "${CMAKE_CURRENT_LIST_DIR}/traced_assert.h"
        "${CMAKE_CURRENT_LIST_DIR}/accuracy.h"
        "${CMAKE_CURRENT_LIST_DIR}/cartesian_product.h"
        "${CMAKE_CURRENT_LIST_DIR}/cartesian_product_score_based.h"
        "${CMAKE_CURRENT_LIST_DIR}/probability.h"
        "${CMAKE_CURRENT_LIST_DIR}/rng.h"
        "${CMAKE_CURRENT_LIST_DIR}/csvimport.h"
        "${CMAKE_CURRENT_LIST_DIR}/osgtools.h"
        "${CMAKE_CURRENT_LIST_DIR}/render.h"
        "${CMAKE_CURRENT_LIST_DIR}/property.h"
        "${CMAKE_CURRENT_LIST_DIR}/propertylist.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/config.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/global.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/files.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/format.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/logger.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/number.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/json_tools.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/system.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timeconv.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/stringconv.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/event_log.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/stringmat.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/logmodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timewindow.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/traced_assert.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/msghandler.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/accuracy.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/probability.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rng.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/csvimport.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/osgtools.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/property.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/propertylist.cpp"
)

target_include_directories(compass_util PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(compass_util
    PUBLIC
        Qt5::Widgets
        Qt5::Core
        ${Boost_LIBRARIES}
        ${LOG4CPP_LIBRARIES}
        ${OPENSSL_CRYPTO_LIBRARY}
        ${TBB_LIBRARIES}
        TBB::tbb
        Threads::Threads
        ${CMAKE_DL_LIBS}
        Eigen3::Eigen
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEARTH_LIBRARY}
        stdc++fs # GCC 8 (Debian 10) needs this for std::filesystem; harmless no-op on GCC 9+
)

compass_automoc(compass_util)
