add_executable(compass_tests
    test_main.cpp
)

# Include tests from library subfolders
include("${CMAKE_SOURCE_DIR}/src/core/unit_tests/CMakeLists.txt")

target_include_directories(compass_tests PRIVATE
    "${COMPASS_PATH}/lib"
)

target_compile_definitions(compass_tests PRIVATE BUFFER_PEDANTIC_CHECKING_ENABLED)

# The library split introduced circular cross-library symbol references
# (e.g. compass_db ↔ compass_eval, compass_reconstruction ↔ experimental
# sources in compass, compass_core ↔ experimental sources).  Wrapping with
# --start-group / --end-group makes the linker rescan archives until all
# symbols are resolved, regardless of link order.
target_link_libraries(compass_tests PRIVATE
    -Wl,--start-group
    compass_app compass compass_core compass_db compass_reconstruction
    compass_import compass_gui compass_filter compass_task compass_eval compass_view
    -Wl,--end-group
    ${jASTERIX_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_test(NAME compass_tests COMMAND compass_tests)
