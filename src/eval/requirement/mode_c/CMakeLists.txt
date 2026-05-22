target_include_directories(compass_eval PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_eval
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/falseconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/modecfalseconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/false.h"
        "${CMAKE_CURRENT_LIST_DIR}/presentconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/modecpresentconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/present.h"    
        "${CMAKE_CURRENT_LIST_DIR}/correctconfig.h"
        "${CMAKE_CURRENT_LIST_DIR}/modeccorrectconfigwidget.h"
        "${CMAKE_CURRENT_LIST_DIR}/correct.h"
        "${CMAKE_CURRENT_LIST_DIR}/correct_period.h"
        "${CMAKE_CURRENT_LIST_DIR}/rocdcorrect.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/falseconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/modecfalseconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/false.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/presentconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/modecpresentconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/present.cpp"        
        "${CMAKE_CURRENT_LIST_DIR}/correctconfig.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/modeccorrectconfigwidget.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/correct.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/correct_period.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rocdcorrect.cpp"
    )


