include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_report
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/latexvisitor.h"
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/latexvisitor.cpp"
)
