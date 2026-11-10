
target_include_directories(compass_core PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_core
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/datasource_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourcewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourceselectioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidgetbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcestoolwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesusewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/deletedatadialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecompoundcoverage.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatus.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatuswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatustoolwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/sensorstatustracker.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/datasource_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourcewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidgetbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcestoolwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesusewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/deletedatadialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecompoundcoverage.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatus.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatuswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesstatustoolwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sensorstatustracker.cpp"
)
