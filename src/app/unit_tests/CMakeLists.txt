# test plugin modules for test_plugin_manager.cpp, built like the real plugins (MODULE, no "lib" prefix,
# linked to compass_plugin_sdk), but into their own directory. compass_plugin_wrongname contains the same
# plugin under another file name (name check).
set(COMPASS_TEST_PLUGIN_DIR "${CMAKE_BINARY_DIR}/test_plugins")

foreach(_test_plugin compass_plugin_testplugin compass_plugin_wrongname)
    add_library(${_test_plugin} MODULE "${CMAKE_CURRENT_LIST_DIR}/testplugin.cpp")
    target_link_libraries(${_test_plugin} PRIVATE compass_plugin_sdk)
    target_link_options(${_test_plugin} PRIVATE "-Wl,--no-undefined")
    set_target_properties(${_test_plugin} PROPERTIES
        PREFIX ""
        AUTOMOC OFF
        LIBRARY_OUTPUT_DIRECTORY "${COMPASS_TEST_PLUGIN_DIR}"
    )
    add_dependencies(compass_tests ${_test_plugin})
endforeach()

target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_plugin_manager.cpp
)

target_compile_definitions(compass_tests PRIVATE COMPASS_TEST_PLUGIN_DIR="${COMPASS_TEST_PLUGIN_DIR}")
