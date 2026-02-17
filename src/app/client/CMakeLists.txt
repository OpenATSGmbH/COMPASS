
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_app
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/client.h"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow.h"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_file.h"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_import.h"
        "${CMAKE_CURRENT_LIST_DIR}/toolbox.h"
        "${CMAKE_CURRENT_LIST_DIR}/toolboxwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/toolboxdefs.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/client.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_file.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_import.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/toolbox.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/toolboxwidget.cpp"
)

add_executable ( compass_client "${CMAKE_CURRENT_LIST_DIR}/main.cpp")

# The library split introduced circular cross-library symbol references
# (e.g. compass_db ↔ compass_eval, compass_reconstruction ↔ experimental
# sources in compass, compass_core ↔ experimental sources).  Wrapping with
# --start-group / --end-group makes the linker rescan archives until all
# symbols are resolved, regardless of link order.
target_link_libraries ( compass_client
    -Wl,--start-group
    compass_app compass compass_core compass_db compass_reconstruction
    compass_import compass_gui compass_filter compass_task compass_eval compass_view
    -Wl,--end-group
    ${jASTERIX_LIBRARIES}
    ${Boost_LIBRARIES}
)

install (TARGETS compass_client DESTINATION bin)
