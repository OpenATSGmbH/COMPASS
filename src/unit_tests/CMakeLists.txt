add_executable(compass_tests
    test_main.cpp
)

# Tests have no Q_OBJECT classes of their own. Disable AUTOMOC to prevent
# re-MOCing headers from linked libraries (their .cpp files already include
# their own moc_*.cpp, so a second MOC would cause duplicate symbols).
set_target_properties(compass_tests PROPERTIES AUTOMOC OFF)

# Include tests from library subfolders
include("${CMAKE_SOURCE_DIR}/src/config/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/core/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/core/context/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/eval/requirement/detection/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/task/analyze/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/task/analyze/mlat/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/task/reconstructor/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/filter/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/reconstruction/unit_tests/CMakeLists.txt")
include("${CMAKE_SOURCE_DIR}/src/gui/unit_tests/CMakeLists.txt")

target_include_directories(compass_tests PRIVATE
    "${COMPASS_PATH}/lib"
)

target_compile_definitions(compass_tests PRIVATE BUFFER_PEDANTIC_CHECKING_ENABLED)

target_link_libraries(compass_tests PRIVATE
    compass_app
    compass_config compass_core compass_reconstruction
    compass_gui compass_filter compass_task
    compass_eval compass_view compass_report
    # Repeated: static archive link order - COMPASS:: methods in compass_app
    # and compass_core symbols are needed by libs listed above (eval, view, task, etc.)
    compass_app compass_core
    ${jASTERIX_LIBRARIES}
    ${Boost_LIBRARIES}
    Qt5::Widgets Qt5::Core Qt5::OpenGL Qt5::Charts Qt5::Test
    ${OPENSCENEGRAPH_LIBRARIES}
    ${OSGEARTH_LIBRARY}
)

if(EXPERIMENTAL_SRC)
    target_link_libraries(compass_tests PRIVATE
        compass_osg_utils compass_docx compass_reconst_adv
        compass_geo_view compass_rtcmd_ext compass_test_lab
        # Repeated: experimental libs reference COMPASS:: methods and core/task symbols
        compass_app compass_core compass_reconstruction compass_task
    )
endif()

add_test(NAME compass_tests COMMAND compass_tests)
