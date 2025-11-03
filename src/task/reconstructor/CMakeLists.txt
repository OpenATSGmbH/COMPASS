
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortask.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortarget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorassociatorbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructor.h"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructorwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructorassociationwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/simpleassociator.h"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatorsettings.h"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatordefs.h"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculator.h"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatorwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatorannotations.h"
        "${CMAKE_CURRENT_LIST_DIR}/simpleaccuracyestimator.h"
        "${CMAKE_CURRENT_LIST_DIR}/accuracyestimatorbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskclassificationwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskanalysiswidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructormainwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorsectorwidget.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortask.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortarget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorassociatorbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructorwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/simplereconstructorassociationwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/simpleassociator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatorwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/referencecalculatorannotations.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/simpleaccuracyestimator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/accuracyestimatorbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskclassificationwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructortaskanalysiswidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructormainwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reconstructorsectorwidget.cpp"
)
