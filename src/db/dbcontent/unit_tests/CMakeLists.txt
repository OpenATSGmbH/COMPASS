target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_sortedlistdiff.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_calculation_plan.cpp
)

# test_calculation_plan.cpp reads conf/default
target_compile_definitions(compass_tests PRIVATE COMPASS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
