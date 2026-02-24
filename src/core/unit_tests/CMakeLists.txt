target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_cpp17.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_number.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_projection.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_nullablevector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_buffer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_dict_nullable_vector.cpp
)
