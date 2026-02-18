
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_helpers.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_macros.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_chain.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_runner.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_runner_stash.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_wait_condition.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_manager.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_registry.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_shell.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_string.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_result.h"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_response.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_helpers.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_chain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_runner.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_runner_stash.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_wait_condition.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_manager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_shell.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_string.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rtcommand_response.cpp"
)
