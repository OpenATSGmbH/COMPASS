cmake_minimum_required(VERSION 3.16)

project(deps_test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# COMPASS find modules (cmake_modules), as used by the COMPASS build
set(COMPASS_CMAKE_MODULES "" CACHE PATH "COMPASS cmake_modules directory")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${COMPASS_CMAKE_MODULES})

# same find calls as the COMPASS CMakeLists.txt
find_package(Boost 1.73.0 REQUIRED COMPONENTS regex program_options filesystem iostreams thread stacktrace_backtrace)
find_package(Eigen3 3.2 REQUIRED)
find_package(LOG4CPP REQUIRED)
find_package(GDAL REQUIRED)
find_package(LibArchive REQUIRED)
# COMPASS FindTBB.cmake searches the library name tbb, oneTBB on Windows is tbb12: use the oneTBB config
find_package(TBB CONFIG REQUIRED)
set(TBB_LIBRARIES TBB::tbb)
find_package(OpenSSL REQUIRED)
find_package(NemaTode REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(PCAP REQUIRED)
find_package(GeographicLib REQUIRED)

# FindDuckDB.cmake only searches libduckdb.so, the Windows package has duckdb.lib
find_library(DUCKDB_LIBRARIES duckdb REQUIRED)
find_path(DUCKDB_INCLUDE_DIR duckdb.h REQUIRED)

foreach(var Boost_LIBRARIES EIGEN3_INCLUDE_DIR LOG4CPP_LIBRARIES GDAL_LIBRARIES LibArchive_LIBRARIES
            TBB_LIBRARIES OPENSSL_CRYPTO_LIBRARY PCAP_LIBRARY GeographicLib_LIBRARIES DUCKDB_LIBRARIES)
    message(STATUS "${var}: ${${var}}")
endforeach()

add_executable(deps_test main.cpp)
target_include_directories(deps_test PRIVATE
    ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${LOG4CPP_INCLUDE_DIR} ${GDAL_INCLUDE_DIRS}
    ${LibArchive_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR} ${GeographicLib_INCLUDE_DIRS}
    ${DUCKDB_INCLUDE_DIR})
# log4cpp/config-MinGW32.h, see Dockerfile_win_deb12, stacktrace defines as in src/app/client
target_compile_definitions(deps_test PRIVATE LOG4CPP_HAVE_INT64_T BOOST_STACKTRACE_USE_BACKTRACE BOOST_STACKTRACE_LINK)
target_link_libraries(deps_test
    ${Boost_LIBRARIES} ${LOG4CPP_LIBRARIES} ${GDAL_LIBRARIES} ${LibArchive_LIBRARIES} ${TBB_LIBRARIES}
    OpenSSL::Crypto NemaTode::NemaTode ${GeographicLib_LIBRARIES} ${DUCKDB_LIBRARIES}
    Threads::Threads ws2_32 dbghelp)

# pcap only links (pcap.dll of the own libpcap build)
add_executable(pcap_link_test pcap_main.cpp)
target_include_directories(pcap_link_test PRIVATE ${PCAP_INCLUDE_DIR})
target_link_libraries(pcap_link_test ${PCAP_LIBRARY})

install(TARGETS deps_test pcap_link_test RUNTIME DESTINATION .)
