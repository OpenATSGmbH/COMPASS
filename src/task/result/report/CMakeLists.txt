
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/reportdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/section.h"
        "${CMAKE_CURRENT_LIST_DIR}/treeitem.h"
        "${CMAKE_CURRENT_LIST_DIR}/treemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/report.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportitem.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectionid.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontent.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontenttext.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontenttable.h"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontentfigure.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexport.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporter.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterjson.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterlatex.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterdocx.h"
        "${CMAKE_CURRENT_LIST_DIR}/reportexportdialog.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/reportdefs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/section.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/treeitem.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/treemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/report.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportitem.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectionid.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontent.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontenttext.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontenttable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sectioncontentfigure.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexport.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterjson.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterlatex.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexporterdocx.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reportexportdialog.cpp"
    )
