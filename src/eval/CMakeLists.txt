add_library(compass_eval "")

include("${CMAKE_CURRENT_LIST_DIR}/data/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/requirement/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/results/CMakeLists.txt")
include("${CMAKE_CURRENT_LIST_DIR}/standard/CMakeLists.txt")

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationsettings.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationcalculator.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationmaintabwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtabwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationreporttabwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdatasourcewidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationsectorwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluation_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtimestampconditionsdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetfilterdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetexcludedtimewindowsdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetexcludedrequirementsdialog.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdefs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationsettings.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationcalculator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationmaintabwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationstandardtabwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationreporttabwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationdatasourcewidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationsectorwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluation_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtimestampconditionsdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetfilterdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetexcludedtimewindowsdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/evaluationtargetexcludedrequirementsdialog.cpp"
)

target_include_directories(compass_eval PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_eval
    PUBLIC
        compass_core
        compass_reconstruction
        compass_task
        compass_gui
)

# eval sources include headers from app and view - circular deps
target_include_directories(compass_eval PRIVATE
    "${CMAKE_SOURCE_DIR}/src/app/client"
    "${CMAKE_SOURCE_DIR}/src/view"
    "${CMAKE_SOURCE_DIR}/src/view/viewbase"
    "${CMAKE_SOURCE_DIR}/src/view/points"
    "${CMAKE_SOURCE_DIR}/src/view/histogramview"
    "${CMAKE_SOURCE_DIR}/src/view/scatterplotview"
    "${CMAKE_SOURCE_DIR}/src/view/gridview"
)

compass_automoc(compass_eval)
