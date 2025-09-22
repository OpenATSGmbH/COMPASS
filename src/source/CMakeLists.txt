
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/datasourcemanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcebase.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurationdatasource.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasource.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasource_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcelineinfo.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourcewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourceselectioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcestoolwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesusewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceeditwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcetablemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesconfigurationdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/dstypeselectioncombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecreatedialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecompoundcoverage.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/datasourcemanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcebase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurationdatasource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasource.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasource_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcelineinfo.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbdatasourcewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcestoolwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesusewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceeditwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcetablemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcesconfigurationdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecreatedialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourcecompoundcoverage.cpp"
)


