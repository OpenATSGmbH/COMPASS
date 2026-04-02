
target_include_directories(compass_app PUBLIC
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
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/client.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_file.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mainwindow_commands_import.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/toolbox.cpp"
)

add_executable ( compass_client
    "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/cxa_throw_hook.cpp"
)

# export symbols so backtrace_symbols_fd() can resolve function names on crash
# No Q_OBJECT sources in this target (they live in compass_app). Disable AUTOMOC
# to prevent re-MOCing headers from linked libraries (whose .cpp files already
# include their own moc_*.cpp).
set_target_properties(compass_client PROPERTIES ENABLE_EXPORTS TRUE AUTOMOC OFF)
target_compile_definitions(compass_client PRIVATE
    BOOST_STACKTRACE_USE_BACKTRACE
    BOOST_STACKTRACE_LINK
)

target_link_libraries ( compass_client
    compass_app
    compass_config compass_core compass_reconstruction
    compass_gui compass_filter compass_task
    compass_eval compass_view compass_report
    # Repeated: static archive link order — COMPASS:: methods in compass_app
    # and compass_core symbols are needed by libs listed above (eval, view, task, etc.)
    compass_app compass_core
    ${jASTERIX_LIBRARIES}
    ${Boost_LIBRARIES}
    Qt5::Widgets Qt5::Core Qt5::OpenGL Qt5::Charts Qt5::Test
    ${OPENSCENEGRAPH_LIBRARIES}
    ${OSGEARTH_LIBRARY}
)

if(EXPERIMENTAL_SRC)
    target_link_libraries(compass_client
        compass_osg_utils compass_docx compass_reconst_adv
        compass_geo_view compass_rtcmd_ext compass_test_lab
        # Repeated: experimental libs reference COMPASS:: methods and core/task symbols
        compass_app compass_core compass_reconstruction compass_task
    )
    target_include_directories(compass_app PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/rtcommand"
    )
endif()

install (TARGETS compass_client DESTINATION bin)
