
add_executable ( compass_handler "${CMAKE_CURRENT_LIST_DIR}/main.cpp")

if(WIN32)
    # application manifest (see the top-level CMakeLists.txt)
    add_dependencies(compass_handler compass_windows_rc)
    target_link_libraries(compass_handler "${COMPASS_WINDOWS_RC}")
endif()

target_include_directories(compass_handler PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_link_libraries ( compass_handler
    Qt5::Widgets
    Qt5::Core
    Qt5::OpenGL
    Qt5::Charts
    Qt5::Test
    ${Boost_LIBRARIES}
)

install (TARGETS compass_handler DESTINATION bin)
