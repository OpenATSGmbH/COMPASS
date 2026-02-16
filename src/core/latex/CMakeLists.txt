include("${CMAKE_CURRENT_LIST_DIR}/visitor/CMakeLists.txt")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/latexcontent.h"
        "${CMAKE_CURRENT_LIST_DIR}/latexdocument.h"
        "${CMAKE_CURRENT_LIST_DIR}/latexsection.h"
        "${CMAKE_CURRENT_LIST_DIR}/latextable.h"
        "${CMAKE_CURRENT_LIST_DIR}/lateximage.h"
        "${CMAKE_CURRENT_LIST_DIR}/latextext.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/latexcontent.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latexdocument.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latexsection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latextable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/lateximage.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/latextext.cpp"
)


