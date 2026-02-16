
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
target_link_libraries ( compass_client compass_app)
