# Use the Linux Qt5 host tools (moc, uic, rcc) with the Windows Qt5 libraries in cross builds.
# The prebuilt Windows Qt only has Windows executables for these tools.
# Include this file before find_package(Qt5 ...).

foreach(tool moc uic rcc)
    if(NOT TARGET Qt5::${tool})
        add_executable(Qt5::${tool} IMPORTED)
        set_target_properties(Qt5::${tool} PROPERTIES IMPORTED_LOCATION /usr/lib/qt5/bin/${tool})
    endif()
endforeach()
