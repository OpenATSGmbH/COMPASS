add_library(compass_core "")

include("${CMAKE_CURRENT_LIST_DIR}/base/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/property/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/buffer/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/config/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/util/CMakeLists.txt")
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
include("${CMAKE_CURRENT_LIST_DIR}/test/CMakeLists.txt")

# DB layer (merged into compass_core)
include("${CMAKE_SOURCE_DIR}/src/db/CMakeLists.txt")

target_sources(compass_core
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/compass.h"
    "${CMAKE_CURRENT_LIST_DIR}/appmode.h"
)

target_include_directories(compass_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

# db layer files merged into compass_core that depend on higher-level modules.
# PRIVATE to avoid AUTOMOC duplication — these dirs contain Q_OBJECT headers
# that are already MOC'd by their own targets.
target_include_directories(compass_core PRIVATE
    "${CMAKE_SOURCE_DIR}/src/app/client"
    "${CMAKE_SOURCE_DIR}/src/eval"
    "${CMAKE_SOURCE_DIR}/src/eval/data"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/base"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/detection"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/dubious"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/extra"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/generic"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/identification"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/mode_a"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/mode_c"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/mom"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/position"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/speed"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/status"
    "${CMAKE_SOURCE_DIR}/src/eval/requirement/trackangle"
    "${CMAKE_SOURCE_DIR}/src/eval/results"
    "${CMAKE_SOURCE_DIR}/src/eval/results/base"
    "${CMAKE_SOURCE_DIR}/src/eval/results/detection"
    "${CMAKE_SOURCE_DIR}/src/eval/results/dubious"
    "${CMAKE_SOURCE_DIR}/src/eval/results/extra"
    "${CMAKE_SOURCE_DIR}/src/eval/results/generic"
    "${CMAKE_SOURCE_DIR}/src/eval/results/identification"
    "${CMAKE_SOURCE_DIR}/src/eval/results/mode_a"
    "${CMAKE_SOURCE_DIR}/src/eval/results/mode_c"
    "${CMAKE_SOURCE_DIR}/src/eval/results/position"
    "${CMAKE_SOURCE_DIR}/src/eval/results/speed"
    "${CMAKE_SOURCE_DIR}/src/eval/results/trackangle"
    "${CMAKE_SOURCE_DIR}/src/eval/standard"
    "${CMAKE_SOURCE_DIR}/src/filter"
    "${CMAKE_SOURCE_DIR}/src/gui"
    "${CMAKE_SOURCE_DIR}/src/import"
    "${CMAKE_SOURCE_DIR}/src/import/asterix"
    "${CMAKE_SOURCE_DIR}/src/import/json"
    "${CMAKE_SOURCE_DIR}/src/reconstruction"
    "${CMAKE_SOURCE_DIR}/src/report"
    "${CMAKE_SOURCE_DIR}/src/report/latex"
    "${CMAKE_SOURCE_DIR}/src/report/latex/visitor"
    "${CMAKE_SOURCE_DIR}/src/task"
    "${CMAKE_SOURCE_DIR}/src/task/assoc"
    "${CMAKE_SOURCE_DIR}/src/task/calc"
    "${CMAKE_SOURCE_DIR}/src/task/import"
    "${CMAKE_SOURCE_DIR}/src/task/import/asterix"
    "${CMAKE_SOURCE_DIR}/src/task/import/gps"
    "${CMAKE_SOURCE_DIR}/src/task/import/json"
    "${CMAKE_SOURCE_DIR}/src/task/import/view_points"
    "${CMAKE_SOURCE_DIR}/src/task/manage"
    "${CMAKE_SOURCE_DIR}/src/task/reconstructor"
    "${CMAKE_SOURCE_DIR}/src/task/result"
    "${CMAKE_SOURCE_DIR}/src/task/result/report"
    "${CMAKE_SOURCE_DIR}/src/view"
    "${CMAKE_SOURCE_DIR}/src/view/viewbase"
    "${CMAKE_SOURCE_DIR}/src/view/points"
    "${CMAKE_SOURCE_DIR}/src/view/points/report"
    "${CMAKE_SOURCE_DIR}/src/view/gridview"
    "${CMAKE_SOURCE_DIR}/src/view/histogramview"
    "${CMAKE_SOURCE_DIR}/src/view/scatterplotview"
    "${CMAKE_SOURCE_DIR}/src/view/tableview"
)

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
        ${SQLITE3_LIBRARIES}
        ${DUCKDB_LIBRARIES}
        Eigen3::Eigen
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEARTH_LIBRARY}
)

compass_automoc(compass_core)
