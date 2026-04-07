
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/data_source.h"
        "${CMAKE_CURRENT_LIST_DIR}/fft.h"
        "${CMAKE_CURRENT_LIST_DIR}/asterix_decoding_config.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_serializer.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_diff.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_manager.h"
        "${CMAKE_CURRENT_LIST_DIR}/idatasourceprovider.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_create_dialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_select_dialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_delete_dialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_copy_dialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/data_source.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/fft.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/asterix_decoding_config.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_serializer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_diff.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_manager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_create_dialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_select_dialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_delete_dialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/db_context_copy_dialog.cpp"
)
