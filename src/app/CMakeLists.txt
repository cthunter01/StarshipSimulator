# The StarshipSimulator executable. Shaders are found next to it (bin/shaders).
add_executable(StarshipSimulator
    Application.cpp
    hud.cpp
    main.cpp
    SdlInput.cpp
    sky_loader.cpp)
target_link_libraries(StarshipSimulator
    PRIVATE StarshipSimulator::render StarshipSimulator::physics StarshipSimulator::audio)
add_dependencies(StarshipSimulator StarshipSimulator_data)
# Where to find the downloaded sky data when it is not installed next to the executable.
target_compile_definitions(StarshipSimulator PRIVATE
    STARSHIPSIMULATOR_SKY_DATA_DIR="${STARSHIPSIMULATOR_SKY_DATA_DIR}")
StarshipSimulator_configure_target(StarshipSimulator)

include(GNUInstallDirs)
install(TARGETS StarshipSimulator RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders ${PROJECT_SOURCE_DIR}/data
        DESTINATION ${CMAKE_INSTALL_BINDIR})
if(EXISTS "${STARSHIPSIMULATOR_SKY_DATA_DIR}")
    install(DIRECTORY ${STARSHIPSIMULATOR_SKY_DATA_DIR}/ DESTINATION ${CMAKE_INSTALL_BINDIR}/data/sky
            PATTERN "*.part" EXCLUDE)
endif()
