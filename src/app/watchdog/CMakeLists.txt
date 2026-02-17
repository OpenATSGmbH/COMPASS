
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

#target_sources(compass
#    PUBLIC
#
#    PRIVATE
#)

add_executable ( compass_handler "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries ( compass_handler
    Qt5::Widgets
    Qt5::Core
    Qt5::OpenGL
    Qt5::Charts
    Qt5::Test
    ${Boost_LIBRARIES}
)

install (TARGETS compass_handler DESTINATION bin)
