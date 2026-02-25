include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/docxcontent.h"
        "${CMAKE_CURRENT_LIST_DIR}/docxdocument.h"
        "${CMAKE_CURRENT_LIST_DIR}/docxsection.h"
        "${CMAKE_CURRENT_LIST_DIR}/docxtable.h"
        "${CMAKE_CURRENT_LIST_DIR}/docximage.h"
        "${CMAKE_CURRENT_LIST_DIR}/docxtext.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/docxcontent.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/docxdocument.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/docxsection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/docxtable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/docximage.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/docxtext.cpp"
)
