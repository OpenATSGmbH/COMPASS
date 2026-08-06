target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_timestamp_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_mode3a_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_modec_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_acad_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_acid_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_utn_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_primaryonly_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_adsbquality_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_mlatru_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_tracker_tracknum_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_reftraj_accuracy_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_excluded_timewindows_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_dbfilter_condition.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_filter_clause.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_filter_buffer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_filter_generator_dialog.cpp
)
