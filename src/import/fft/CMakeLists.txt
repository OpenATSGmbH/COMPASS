
include_directories (
    "${CMAKE_CURRENT_LIST_DIR}"
    )

target_sources(compass_import
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/fftmanager.h"
        "${CMAKE_CURRENT_LIST_DIR}/fftbase.h"
        "${CMAKE_CURRENT_LIST_DIR}/configurationfft.h"
        "${CMAKE_CURRENT_LIST_DIR}/dbfft.h"
        "${CMAKE_CURRENT_LIST_DIR}/ffttablemodel.h"
        "${CMAKE_CURRENT_LIST_DIR}/fftsconfigurationdialog.h"
        "${CMAKE_CURRENT_LIST_DIR}/ffteditwidget.h"
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/fftmanager.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/fftbase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/configurationfft.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/dbfft.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ffttablemodel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/fftsconfigurationdialog.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ffteditwidget.cpp"
        )


