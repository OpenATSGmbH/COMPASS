
target_include_directories(compass_task PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(compass_task
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/datasourceinspectorbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/inspectorsettingsbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/targetreport3dgrid.h"
        "${CMAKE_CURRENT_LIST_DIR}/analysisdataset.h"
        "${CMAKE_CURRENT_LIST_DIR}/analyzedatasourcetask.h"
        "${CMAKE_CURRENT_LIST_DIR}/analyzedatasourcedialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/analyze_commands.h"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceanalysistaskresult.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/datasourceinspectorbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/inspectorsettingsbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/targetreport3dgrid.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/analysisdataset.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/analyzedatasourcetask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/analyzedatasourcedialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/analyze_commands.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/datasourceanalysistaskresult.cpp"
)

include("${CMAKE_CURRENT_LIST_DIR}/mlat/CMakeLists.txt")

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_task PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat"
    )
    target_sources(compass_task
        PRIVATE
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlataccuracyinspector.h"
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlataccuracyinspector.cpp"
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlatrucoverageinspector.h"
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlatrucoverageinspector.cpp"
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlatrueffectinspector.h"
            "${CMAKE_SOURCE_DIR}/experimental_src/analysis/mlat/mlatrueffectinspector.cpp"
    )
endif()
