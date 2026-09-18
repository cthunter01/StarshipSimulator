# Unit tests link only the core library, so they run anywhere (no window or GPU needed).
add_executable(StarshipSimulator_tests
    app_options_test.cpp
    camera_test.cpp
    habitat_mesher_test.cpp
    habitat_test.cpp
    mesh_test.cpp
    physics_test.cpp
    procgen_test.cpp
    scenario_test.cpp
    sim_clock_test.cpp
    spirv_reflect_test.cpp
    uniforms_test.cpp)
target_link_libraries(StarshipSimulator_tests PRIVATE StarshipSimulator::core GTest::gtest_main)
# spirv_reflect_test checks every compiled shader against SDL_GPU's binding rules.
add_dependencies(StarshipSimulator_tests StarshipSimulator_shaders)
target_compile_definitions(StarshipSimulator_tests PRIVATE
    STARSHIPSIMULATOR_SHADER_DIR="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders"
    STARSHIPSIMULATOR_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
StarshipSimulator_configure_target(StarshipSimulator_tests)

include(GoogleTest)
# PRE_TEST: list the tests when ctest runs, not at build time (the binary may be sanitized or cross-compiled).
gtest_discover_tests(StarshipSimulator_tests DISCOVERY_MODE PRE_TEST)

# The core library must not depend on SDL, ImGui, or the renderer.
add_test(NAME layering
    COMMAND ${CMAKE_COMMAND} -DROOT=${PROJECT_SOURCE_DIR} -P ${PROJECT_SOURCE_DIR}/cmake/CheckLayering.cmake)
