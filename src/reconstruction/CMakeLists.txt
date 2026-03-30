add_library(compass_reconstruction "")

target_sources(compass_reconstruction
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/kalman_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_tools.h"
        "${CMAKE_CURRENT_LIST_DIR}/measurement.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter_linear.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter_um2d.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_projection.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_interface.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_interface_umkalman2d.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_estimator.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_online_tracker.h"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_chain.h"
        "${CMAKE_CURRENT_LIST_DIR}/reconstruction_defs.h"
        "${CMAKE_CURRENT_LIST_DIR}/spline_interpolator.h"
        "${CMAKE_CURRENT_LIST_DIR}/gaussian_pdf.h"

    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/kalman_defs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_tools.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter_linear.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_filter_um2d.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/measurement.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_projection.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_interface.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_interface_umkalman2d.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_estimator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_online_tracker.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kalman_chain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/spline_interpolator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/gaussian_pdf.cpp"
)

target_include_directories(compass_reconstruction PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

if(EXPERIMENTAL_SRC)
    target_include_directories(compass_reconstruction PRIVATE
        "${CMAKE_SOURCE_DIR}/experimental_src/reconstruction/kalman"
    )
endif()

target_link_libraries(compass_reconstruction
    PUBLIC
        compass_core
        Eigen3::Eigen
)

compass_automoc(compass_reconstruction)
