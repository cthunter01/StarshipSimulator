#pragma once

#include <filesystem>
#include <string_view>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// Loads compiled SPIR-V shaders (build/<preset>/bin/shaders/*.spv). Resource counts for SDL_GPU
/// come from reflecting the SPIR-V, and the binding layout is validated against SDL_GPU's
/// convention, so a mismatch is a clear error at load time instead of a crash or silently wrong
/// rendering. On a device that takes Metal Shading Language instead (macOS), each shader is
/// translated from its SPIR-V as it loads (gpu::translateToMetal).
class ShaderLibrary
{
public:
    /// Throws std::runtime_error if the device takes neither SPIR-V nor MSL.
    ShaderLibrary(SDL_GPUDevice* device, std::filesystem::path directory);

    /// Loads "<directory>/<name>.spv", e.g. load("habitat.frag"). Throws std::runtime_error.
    [[nodiscard]] GpuShader load(std::string_view name) const;

    [[nodiscard]] const std::filesystem::path& directory() const { return directory_; }

private:
    SDL_GPUDevice*        device_;
    std::filesystem::path directory_;
    SDL_GPUShaderFormat   format_;  // SPIR-V, or MSL on Metal
};

/// The directory holding the compiled shaders: next to the executable.
[[nodiscard]] std::filesystem::path defaultShaderDirectory();

}  // namespace StarshipSimulator
