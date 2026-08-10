target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_kalman_filter_um2d.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_spline_interpolator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_kalman_estimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_kalman_chain.cpp
)

target_include_directories(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
