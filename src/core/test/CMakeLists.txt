
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_common.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_testable.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_conversions.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_event_injections.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_widget_setters.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_widget_getters.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_setget_native.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_setget.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_inject.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_find.h"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_cmd.h"

PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_event_injections.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_setget_native.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_setget.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_inject.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_find.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ui_test_cmd.cpp"
)


