#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"

// SDL_GPU runs on Metal on macOS, which takes Metal Shading Language rather than SPIR-V. Our
// shaders are written and compiled once, for Vulkan; this turns the compiled SPIR-V into MSL when
// the app runs on Metal (SPIRV-Cross does the translating).
namespace StarshipSimulator::gpu
{

/// Where one SPIR-V resource (descriptor set and binding) goes on Metal, which numbers buffers,
/// textures and samplers separately. Indices a resource does not use are left at 0.
struct MetalBinding
{
    std::uint32_t set     = 0;
    std::uint32_t binding = 0;
    std::uint32_t buffer  = 0;  // [[buffer(n)]]
    std::uint32_t texture = 0;  // [[texture(n)]]
    std::uint32_t sampler = 0;  // [[sampler(n)]]
};

/// SDL_GPU's Metal layout for a vertex or fragment shader laid out for Vulkan (validateSdlGpuLayout
/// passes): [[texture]] sampled textures, then storage textures; [[sampler]] one per sampled
/// texture, same index; [[buffer]] uniform buffers, then storage buffers. Vertex buffers come in
/// through [[stage_in]] (SDL puts them at [[buffer(14)]] and up). Each kind keeps its Vulkan order.
[[nodiscard]] std::vector<MetalBinding> metalBindings(const ShaderReflection& reflection);

/// A vertex or fragment shader in MSL.
struct MetalShader
{
    std::string source;
    std::string entryPoint;  // SPIRV-Cross renames main, which Metal reserves, to main0
};

/// Translates a SPIR-V vertex or fragment shader (with its own reflection, from reflectSpirv) into
/// MSL 2.1 for macOS, its resources numbered as metalBindings() says. Compute shaders are refused.
[[nodiscard]] std::expected<MetalShader, std::string> translateToMetal(
    std::span<const std::uint32_t> spirv, const ShaderReflection& reflection);

}  // namespace StarshipSimulator::gpu
