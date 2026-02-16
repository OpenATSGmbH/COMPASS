
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        #"${CMAKE_CURRENT_LIST_DIR}/treeitem.h"
        #"${CMAKE_CURRENT_LIST_DIR}/treemodel.h"
        #"${CMAKE_CURRENT_LIST_DIR}/rootitem.h"
        #"${CMAKE_CURRENT_LIST_DIR}/section.h"
        "${CMAKE_CURRENT_LIST_DIR}/evalsectionid.h"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontent.h"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontenttext.h"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontenttable.h"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontentfigure.h"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgenerator.h"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgeneratordialog.h"
    PRIVATE
        #"${CMAKE_CURRENT_LIST_DIR}/treeitem.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/treemodel.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/rootitem.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/section.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evalsectionid.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontent.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontenttext.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontenttable.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/sectioncontentfigure.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgenerator.cpp"
        #"${CMAKE_CURRENT_LIST_DIR}/pdfgeneratordialog.cpp"
    )


