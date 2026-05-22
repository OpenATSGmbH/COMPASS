add_library(compass_gui "")

target_sources(compass_gui
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/autoresumedialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/invalidqlineedit.h"
        "${CMAKE_CURRENT_LIST_DIR}/textfieldhexvalidator.h"
        "${CMAKE_CURRENT_LIST_DIR}/textfieldoctvalidator.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeedit.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider.h"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider_p.h"
        "${CMAKE_CURRENT_LIST_DIR}/formatselectionwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datatypeformatselectionwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/groupbox.h"
        "${CMAKE_CURRENT_LIST_DIR}/selectdatasourceswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/textinputdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/fileimportdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/colorscaleselection.h"
        "${CMAKE_CURRENT_LIST_DIR}/propertyvalueedit.h"
        "${CMAKE_CURRENT_LIST_DIR}/logwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/popupmenu.h"
        "${CMAKE_CURRENT_LIST_DIR}/timewindowcollectionwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/timewindowdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/noscrollqspinbox.h"
        "${CMAKE_CURRENT_LIST_DIR}/questiondialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/autoresumedialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rangeedit.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rangeslider.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/formatselectionwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datatypeformatselectionwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/groupbox.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/selectdatasourceswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/textinputdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/fileimportdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/colorscaleselection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/propertyvalueedit.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/logwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/popupmenu.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timewindowcollectionwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timewindowdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/questiondialog.cpp"
)

target_include_directories(compass_gui PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_gui
    PUBLIC
        compass_core
        Qt5::Widgets
        Qt5::Charts
)

compass_automoc(compass_gui)
