# The SDL_GPU renderer: device, shaders, render targets, passes and the ImGui layer.
add_library(StarshipSimulator_render STATIC
    DynamicBuffer.cpp
    GpuBirds.cpp
    GpuDevice.cpp
    GpuLandscape.cpp
    GpuPeople.cpp
    GpuProps.cpp
    GpuSettlements.cpp
    GpuTransit.cpp
    GpuTrees.cpp
    GpuWorld.cpp
    ImGuiLayer.cpp
    pipeline.cpp
    Renderer.cpp
    RenderTargets.cpp
    ShaderLibrary.cpp
    ShadowMap.cpp
    texture.cpp
    upload.cpp
    passes/habitat_passes.cpp
    passes/MarkerPass.cpp
    passes/sky_passes.cpp
    passes/TonemapPass.cpp
    passes/town_passes.cpp)
add_library(StarshipSimulator::render ALIAS StarshipSimulator_render)
target_link_libraries(StarshipSimulator_render PUBLIC StarshipSimulator::core SDL3::SDL3 StarshipSimulator_imgui)
add_dependencies(StarshipSimulator_render StarshipSimulator_shaders)
StarshipSimulator_configure_target(StarshipSimulator_render)
# SDL_GPU's create-info structs are initialized with designated initializers naming only the fields
# that matter; the rest are zero, which is SDL's documented default.
target_compile_options(StarshipSimulator_render PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-missing-field-initializers>
    $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-missing-designated-field-initializers>)
# Structs holding the 16-byte-aligned uniform blocks (gpu_abi/uniforms.h) are padded to match, which
# is the point; MSVC warns about the padding (C4324) wherever the renderer's headers are included.
target_compile_options(StarshipSimulator_render PUBLIC $<$<CXX_COMPILER_ID:MSVC>:/wd4324>)
