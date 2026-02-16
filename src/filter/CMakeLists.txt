add_library(compass_filter "")

include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_filter
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/dbfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/filtermanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/filtermanagerwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbfiltercondition.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/filtergeneratordialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/filterconditionoperatorcombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/filterconditionresetvaluecombobox.h"
        "${CMAKE_CURRENT_LIST_DIR}/utnfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/utnfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsbqualityfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/adsbqualityfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/acadfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/acadfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/acidfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/acidfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/mode3afilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/mode3afilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/modecfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/modecfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/primaryonlyfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/primaryonlyfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/timestampfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/timestampfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/excludedtimewindowsfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/excludedtimewindowsfilterwidget.h"        
        "${CMAKE_CURRENT_LIST_DIR}/trackertracknumberfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/trackertracknumberfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/reftrajaccuracyfilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/reftrajaccuracyfilterwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/mlatrufilter.h"
        "${CMAKE_CURRENT_LIST_DIR}/mlatrufilterwidget.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dbfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbfiltercondition.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/filtermanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/filtermanagerwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/filtergeneratordialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/utnfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/utnfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsbqualityfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/adsbqualityfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acadfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acadfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acidfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/acidfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mode3afilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mode3afilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/modecfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/modecfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/primaryonlyfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/primaryonlyfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timestampfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/timestampfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/excludedtimewindowsfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/excludedtimewindowsfilterwidget.cpp"                
        "${CMAKE_CURRENT_LIST_DIR}/trackertracknumberfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trackertracknumberfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reftrajaccuracyfilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/reftrajaccuracyfilterwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mlatrufilter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/mlatrufilterwidget.cpp"
)

target_include_directories(compass_filter PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(compass_filter
    PUBLIC
        compass_core
        compass_db
        compass_gui
)
