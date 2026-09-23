# Unit tests link the core and physics libraries, so they run anywhere (no window or GPU needed).
add_executable(StarshipSimulator_tests
    almanac_tests.cpp
    app_options_tests.cpp
    assets_tests.cpp
    astro_tests.cpp
    birds_tests.cpp
    buildings_tests.cpp
    camera_tests.cpp
    clouds_tests.cpp
    determinism_tests.cpp
    habitat_mesher_tests.cpp
    habitat_tests.cpp
    kalpana_tests.cpp
    land_layout_tests.cpp
    mesh_tests.cpp
    metal_shader_tests.cpp
    parse_number_tests.cpp
    people_tests.cpp
    physics_tests.cpp
    PhysicsWorldTests.cpp
    procgen_tests.cpp
    scenario_tests.cpp
    settlements_tests.cpp
    SimClockTests.cpp
    SoundscapeTests.cpp
    spirv_reflect_tests.cpp
    star_catalog_tests.cpp
    terrain_tests.cpp
    tour_tests.cpp
    transit_tests.cpp
    uniforms_tests.cpp
    utf8_path_tests.cpp
    weather_tests.cpp
    wings_tests.cpp)
target_link_libraries(StarshipSimulator_tests
    PRIVATE StarshipSimulator::core StarshipSimulator::physics GTest::gtest_main)
# spirv_reflect_tests checks every compiled shader against SDL_GPU's binding rules.
add_dependencies(StarshipSimulator_tests StarshipSimulator_shaders)
target_compile_definitions(StarshipSimulator_tests PRIVATE
    STARSHIPSIMULATOR_SHADER_DIR="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders"
    STARSHIPSIMULATOR_DATA_DIR="${PROJECT_SOURCE_DIR}/data"
    # Downloaded star catalog and sky maps; the tests that use them skip when they are missing.
    STARSHIPSIMULATOR_SKY_DATA_DIR="${STARSHIPSIMULATOR_SKY_DATA_DIR}")
StarshipSimulator_configure_target(StarshipSimulator_tests)

include(GoogleTest)
# PRE_TEST: list the tests when ctest runs, not at build time (the binary may be sanitized or cross-compiled).
gtest_discover_tests(StarshipSimulator_tests DISCOVERY_MODE PRE_TEST)

# The core library must not depend on SDL, ImGui, or the renderer.
add_test(NAME layering
    COMMAND ${CMAKE_COMMAND} -DROOT=${PROJECT_SOURCE_DIR} -P ${PROJECT_SOURCE_DIR}/cmake/CheckLayering.cmake)
