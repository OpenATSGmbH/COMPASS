
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/variable.h"
        "${CMAKE_CURRENT_LIST_DIR}/variablewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/variableset.h"
        "${CMAKE_CURRENT_LIST_DIR}/variableorderedset.h"
        "${CMAKE_CURRENT_LIST_DIR}/variableorderedsetwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/variabledatatypecombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/variableselectionwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/variableeditdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/variablecreatedialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/metavariable.h"
        "${CMAKE_CURRENT_LIST_DIR}/metavariablewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/metavariableconfigurationdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/metavariabledetailwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/variable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variablewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variableset.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variableorderedset.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variableorderedsetwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variableselectionwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variableeditdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/variablecreatedialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/metavariable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/metavariablewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/metavariableconfigurationdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/metavariabledetailwidget.cpp"
)


