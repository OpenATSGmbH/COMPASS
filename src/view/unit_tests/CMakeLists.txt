target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_liveoverloaddetector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_viewpoint_keys.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_gridbackground.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_render_settings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_legend.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_region.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_feature_json.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_render.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_model.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_grid_render_widget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_figure_selection.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_figure_factory.cpp
)
