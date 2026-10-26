# Qt radar display (PPI scope + track table). Prefers Qt 6, falls back to Qt 5.15.

find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
    set(RADAR_QT_WIDGETS Qt6::Widgets)
else()
    find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets)
    set(RADAR_QT_WIDGETS Qt5::Widgets)
endif()

add_executable(radar_display
    main.cpp
    main_window.cpp
    main_window.hpp
    ppi_widget.cpp
    ppi_widget.hpp)

set_target_properties(radar_display PROPERTIES
    AUTOMOC ON
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# Imported Qt targets put their headers on the SYSTEM include path, so the
# strict project warnings (and -Werror) only apply to our own code.
target_link_libraries(radar_display PRIVATE radar_core ${RADAR_QT_WIDGETS})
target_compile_definitions(radar_display PRIVATE QT_NO_KEYWORDS)
radar_set_project_options(radar_display)
