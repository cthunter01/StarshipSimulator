# The simulation core: math, camera, clocks, controllers, procedural meshes and GPU ABI helpers.
# It must not depend on SDL, ImGui or the renderer (enforced by the 'layering' test).
add_library(StarshipSimulator_core STATIC
    app_options.cpp
    assets/assets.cpp
    astro/astro_time.cpp
    astro/ephemeris.cpp
    astro/sky_objects.cpp
    astro/star_catalog.cpp
    camera.cpp
    frustum.cpp
    sim_clock.cpp
    gpu_abi/color_grade.cpp
    gpu_abi/ground_atlas.cpp
    gpu_abi/spirv_reflect.cpp
    gpu_abi/uniforms.cpp
    habitat/day_schedule.cpp
    habitat/habitat_geometry.cpp
    habitat/habitat_spec.cpp
    habitat/landscape.cpp
    habitat/meridian_profile.cpp
    habitat/metrics.cpp
    habitat/mirror_optics.cpp
    physics/colliders.cpp
    physics/player_controller.cpp
    physics/rotating_frame.cpp
    procgen/buildings.cpp
    procgen/habitat_mesher.cpp
    procgen/hull_mesh.cpp
    procgen/mesh.cpp
    procgen/noise.cpp
    procgen/props.cpp
    procgen/settlements.cpp
    procgen/star_field.cpp
    procgen/terrain_grid.cpp
    procgen/terrain_lod.cpp
    procgen/trees.cpp
    scenario/scenario.cpp)
add_library(StarshipSimulator::core ALIAS StarshipSimulator_core)
target_include_directories(StarshipSimulator_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(StarshipSimulator_core
    PUBLIC glm::glm-header-only
    PRIVATE tomlplusplus::tomlplusplus StarshipSimulator_astronomy StarshipSimulator_stb)
# Every includer must agree on GLM's configuration:
#   XYZW_ONLY: plain x/y/z/w members instead of unions (keeps clang-tidy's union checks quiet)
#   EXPLICIT_CTOR: double -> float vector conversions must be spelled out (no silent precision loss)
#   DEPTH_ZERO_TO_ONE: clip-space depth is 0..1, as in Vulkan/SDL_GPU
target_compile_definitions(StarshipSimulator_core PUBLIC
    GLM_FORCE_XYZW_ONLY GLM_FORCE_EXPLICIT_CTOR GLM_FORCE_DEPTH_ZERO_TO_ONE)
# Deterministic floating point: no fused multiply-add contraction, so GCC and Clang builds generate
# identical worlds from the same seed.
target_compile_options(StarshipSimulator_core PRIVATE -ffp-contract=off)
StarshipSimulator_configure_target(StarshipSimulator_core)
# Terrain generation evaluates noise tens of millions of times: optimize those files even in Debug
# builds, or the app would take half a minute to start.
set_source_files_properties(
    habitat/habitat_geometry.cpp habitat/landscape.cpp habitat/meridian_profile.cpp
    procgen/noise.cpp procgen/settlements.cpp procgen/terrain_grid.cpp procgen/terrain_lod.cpp
    procgen/trees.cpp
    PROPERTIES COMPILE_OPTIONS "$<$<CONFIG:Debug>:-O2>")
