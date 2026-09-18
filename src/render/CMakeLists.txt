# The SDL_GPU renderer: device, shaders, render targets, passes and the ImGui layer.
add_library(StarshipSimulator_render STATIC
    gpu_device.cpp
    gpu_world.cpp
    imgui_layer.cpp
    pipeline.cpp
    render_targets.cpp
    renderer.cpp
    shader_library.cpp
    upload.cpp
    passes/habitat_passes.cpp
    passes/marker_pass.cpp
    passes/tonemap_pass.cpp)
add_library(StarshipSimulator::render ALIAS StarshipSimulator_render)
target_link_libraries(StarshipSimulator_render PUBLIC StarshipSimulator::core SDL3::SDL3 StarshipSimulator_imgui)
add_dependencies(StarshipSimulator_render StarshipSimulator_shaders)
StarshipSimulator_configure_target(StarshipSimulator_render)
# SDL_GPU's create-info structs are initialized with designated initializers naming only the fields
# that matter; the rest are zero, which is SDL's documented default.
target_compile_options(StarshipSimulator_render PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-missing-field-initializers>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-missing-designated-field-initializers>)
