set(COMPASS_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
message("  Path: ${COMPASS_PATH}")

SET(CMAKE_EXPORT_COMPILE_COMMANDS, ON)

cmake_minimum_required(VERSION 3.28)

set ( CMAKE_BUILD_TYPE RelWithDebInfo ) #Debug RelWithDebInfo Release

project( COMPASS )
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "0")

message("  System: ${CMAKE_SYSTEM}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(EXISTS "/.dockerenv")
message("  Platform: Linux (inside docker)")
add_definitions ( -UNDEBUG -Wall -std=c++17 -fext-numeric-literals) # -DQT_NO_KEYWORDS
ELSE()
message("  Platform: Linux (local)")
add_definitions ( -UNDEBUG -Wall -std=c++17 -fno-omit-frame-pointer) # -DQT_NO_KEYWORDS -fext-numeric-literals
ENDIF()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

#-Wno-deprecated-declarations -Wno-ignored-attributes -Wno-misleading-indentation for new compiler and old libs
# -UNDEBUG for enabling asserts in release buils (until testing/error management is improved)

find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5OpenGL)
find_package(Qt5Charts)
find_package(Qt5Test)

message("Qt Widgets version: ${Qt5Widgets_VERSION}")

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Global AUTOMOC is OFF - each target opts in explicitly to avoid
# duplicate MOC generation when a library's .cpp files #include
# Q_OBJECT headers from dependency libraries.
set(CMAKE_AUTOMOC OFF)

# compass_automoc(<target>)
# Collect the target's own .h sources that contain Q_OBJECT or Q_GADGET,
# run qt5_wrap_cpp on them, and add the result as PRIVATE sources.
# Call this AFTER all target_sources() calls for the target.
function(compass_automoc target)
    get_target_property(_sources ${target} SOURCES)
    set(_headers_to_moc)
    foreach(_src IN LISTS _sources)
        if(_src MATCHES "\\.h$")
            file(STRINGS "${_src}" _has_qobject REGEX "Q_OBJECT|Q_GADGET" LIMIT_COUNT 1)
            if(_has_qobject)
                list(APPEND _headers_to_moc "${_src}")
            endif()
        endif()
    endforeach()
    if(_headers_to_moc)
        qt5_wrap_cpp(_moc_srcs ${_headers_to_moc} TARGET ${target})
        target_sources(${target} PRIVATE ${_moc_srcs})
    endif()
endfunction()

#add_definitions(${Qt5Widgets_DEFINITIONS})
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

find_package (Boost 1.73.0 REQUIRED COMPONENTS regex program_options filesystem iostreams thread stacktrace_backtrace)
message("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("  Boost_LIBRARIES: ${Boost_LIBRARIES}")

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS -DBOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING") # suppress internal boost deprecated warnings

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${COMPASS_PATH}/cmake_modules)
message("  CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/usr/local/lib/cmake/;/usr/lib/x86_64-linux-gnu/)
message("  CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package (Eigen3 3.2 REQUIRED)

find_package ( LOG4CPP REQUIRED )
message("  LOG4CPP_INCLUDE_DIR: ${LOG4CPP_INCLUDE_DIR}")
message("  LOG4CPP_LIBRARY: ${LOG4CPP_LIBRARIES}")

find_package ( GDAL REQUIRED )
message("  GDAL_INCLUDE_DIRS: ${GDAL_INCLUDE_DIRS}")
message("  GDAL_LIBRARIES: ${GDAL_LIBRARIES}")

find_package ( LibArchive REQUIRED )
message("  LibArchive_INCLUDE_DIRS: ${LibArchive_INCLUDE_DIRS}")
message("  LibArchive_LIBRARIES: ${LibArchive_LIBRARIES}")

find_package(TBB REQUIRED)
message("  TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
message("  TBB_LIBRARIES: ${TBB_LIBRARIES}")

find_package ( jASTERIX REQUIRED)
message("  jASTERIX found: ${jASTERIX_FOUND}")
message("  jASTERIX_INCLUDE_DIR: ${jASTERIX_INCLUDE_DIR}")
message("  jASTERIX_LIBRARIES: ${jASTERIX_LIBRARIES}")

find_package ( OpenSSL REQUIRED )
message("  OPENSSL_INCLUDE_DIR: ${OPENSSL_INCLUDE_DIR}")
message("  OPENSSL_CRYPTO_LIBRARY: ${OPENSSL_CRYPTO_LIBRARY}")

find_package(NemaTode REQUIRED CONFIG)

find_package(Threads REQUIRED)

find_package(PCAP REQUIRED)

find_package(OpenSceneGraph 3.6.5 REQUIRED osgTerrain osgSim osgDB osgGA osgUtil osgViewer osgText osgParticle osgShadow osgFX)
    # libOpenThreads & libosg automatically searched  osgQt
message("  OSG_INCLUDE_DIR: ${OPENSCENEGRAPH_INCLUDE_DIRS}")

#find_package(osgQt REQUIRED)

#message("  OSGQT_INCLUDE_DIR: ${OSGQT_INCLUDE_DIR}")
#message("  OSGQT_LIBRARIES: ${OSGQT_LIBRARIES}")

find_package(OSGEarth 3.2 REQUIRED)

message("  OSGEARTH_INCLUDE_DIR: ${OSGEARTH_INCLUDE_DIRS}")
message("  OSGEARTH_LIST: ${OSGEARTH_LIBRARY}")

message("  OSGEARTH_INCLUDE_DIRS: ${OSGEARTH_INCLUDE_DIRS}")
message("  OSGEARTH_LIBRARY: ${OSGEARTH_LIBRARY}")

message("OSG libs
    ${OPENSCENEGRAPH_LIBRARIES}"
    #${OSGQT_INCLUDE_DIR}
    )

find_package(GeographicLib REQUIRED)

message("  GeographicLib_INCLUDE_DIRS:  ${GeographicLib_INCLUDE_DIRS}")
message("  GeographicLib_LIBRARIES: ${GeographicLib_LIBRARIES}")

#find_package(Arrow REQUIRED)
#find_package(Parquet REQUIRED)
find_package(DuckDB REQUIRED)

message("Install path: ${CMAKE_INSTALL_PREFIX}")

# Global include directories (for all targets)
include_directories (
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_BINARY_DIR}/include
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${Qt5Charts_INCLUDE_DIRS}
    ${Qt5Test_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${LOG4CPP_INCLUDE_DIR}
    ${GDAL_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${LibArchive_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIR}
    ${jASTERIX_INCLUDE_DIR}
    ${PCAP_INCLUDE_DIR}
    ${GeographicLib_INCLUDE_DIRS}
    #${Arrow_INCLUDE_DIRS}
    #${Parquet_INCLUDE_DIRS}
    ${DUCKDB_INCLUDE_DIR}
    )

link_directories(
    ${Boost_LIBRARY_DIR}
    ${jASTERIX_LIBRARIES}
    )

# Third-party headers
include(lib/CMakeLists.txt)

# src/ root include (for #include "src/..." style paths)
include(src/CMakeLists.txt)

# Detect experimental source early (before component libs that check EXPERIMENTAL_SRC)
set (EXPERIMENTAL_SRC true)
message("Experimental source: ${EXPERIMENTAL_SRC}")

IF (EXPERIMENTAL_SRC)
    IF(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/experimental_src")
        MESSAGE(STATUS "Experimental source found")
    ELSE()
        MESSAGE(STATUS "Experimental source not found found, deactivating")
        set (EXPERIMENTAL_SRC false)
    ENDIF()
ENDIF()

# Component libraries (order matters: dependencies first)
add_subdirectory(src/config)
add_subdirectory(src/core)
add_subdirectory(src/reconstruction)
add_subdirectory(src/gui)
add_subdirectory(src/filter)
add_subdirectory(src/task)
add_subdirectory(src/eval)
add_subdirectory(src/view)
add_subdirectory(src/report)

# Experimental source - creates its own library targets
IF (EXPERIMENTAL_SRC)
    MESSAGE(STATUS "Including experimental source")
    include(experimental_src/CMakeLists.txt)
ENDIF()

# Precompiled headers for faster local incremental builds (not used in Docker/AppImage)
if(NOT EXISTS "/.dockerenv")
    set(COMPASS_PCH_HEADERS
        <string>
        <vector>
        <map>
        <memory>
        <algorithm>
        <functional>
        <QString>
        <QWidget>
        <QObject>
        <QVariant>
        <boost/optional.hpp>
        <boost/geometry.hpp>
        "lib/json.hpp"
    )

    set(COMPASS_PCH_TARGETS
        compass_config compass_core compass_reconstruction
        compass_gui compass_filter compass_task
        compass_eval compass_view compass_report)

    if(EXPERIMENTAL_SRC)
        list(APPEND COMPASS_PCH_TARGETS
            compass_osg_utils compass_docx compass_reconst_adv
            compass_geo_view compass_rtcmd_ext compass_test_lab)
    endif()

    foreach(_target ${COMPASS_PCH_TARGETS})
        set_target_properties(${_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
        target_precompile_headers(${_target} PRIVATE ${COMPASS_PCH_HEADERS})
    endforeach()
endif()

configure_file(src/core/util/global.h.in include/global.h)

# Application executables
add_subdirectory(src/app)

# compass_app links against component libraries inside src/app/CMakeLists.txt

# unit tests
enable_testing()
add_subdirectory(src/unit_tests)

IF (EXPERIMENTAL_SRC)
    add_subdirectory(experimental_src/unit_tests)
ENDIF()

# installation stuff

message("Installing using prefix: ${CMAKE_INSTALL_PREFIX}")
install(DIRECTORY "conf" DESTINATION compass)
install(DIRECTORY "data" DESTINATION compass)

set(COMPASS_INSTALL_LIBS
    compass_config compass_core compass_reconstruction
    compass_gui compass_filter compass_task
    compass_eval compass_view compass_report)
if(EXPERIMENTAL_SRC)
    list(APPEND COMPASS_INSTALL_LIBS
        compass_osg_utils compass_docx compass_reconst_adv
        compass_geo_view compass_rtcmd_ext compass_test_lab)
endif()
install(TARGETS ${COMPASS_INSTALL_LIBS} DESTINATION lib
    PUBLIC_HEADER DESTINATION include/compass)

# compass_client and compass_handler install() moved to src/app/client/ and
# src/app/watchdog/ respectively (CMake <3.13 requires install() in same scope as target)

# build a CPack driven installer package
include (InstallRequiredSystemLibraries)
set (CPACK_RESOURCE_FILE_LICENSE
     "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
include (CPack)
