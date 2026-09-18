# The StarshipSimulator executable. Shaders are found next to it (bin/shaders).
add_executable(StarshipSimulator
    application.cpp
    hud.cpp
    main.cpp
    sdl_input.cpp)
target_link_libraries(StarshipSimulator PRIVATE StarshipSimulator::render)
StarshipSimulator_configure_target(StarshipSimulator)

include(GNUInstallDirs)
install(TARGETS StarshipSimulator RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders DESTINATION ${CMAKE_INSTALL_BINDIR})
