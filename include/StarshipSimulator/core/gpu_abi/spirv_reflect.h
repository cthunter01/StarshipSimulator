#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

// Minimal SPIR-V reflection: just enough to fill SDL_GPUShaderCreateInfo from the shader itself and
// to check that its resource bindings follow SDL_GPU's descriptor-set convention (see CLAUDE.md).
namespace StarshipSimulator::gpu
{

enum class ShaderStage : std::uint8_t
{
    Vertex,
    Fragment,
    Compute,
};

enum class ResourceKind : std::uint8_t
{
    SampledTexture,  // combined image + sampler (sampler2D etc.)
    StorageTexture,  // image2D etc.
    StorageBuffer,   // buffer block
    UniformBuffer,   // uniform block
    Unsupported,     // separate images or samplers, arrays of resources, push constants
};

struct ResourceBinding
{
    std::string   name;
    ResourceKind  kind     = ResourceKind::Unsupported;
    bool          readOnly = true;
    std::uint32_t set      = 0;
    std::uint32_t binding  = 0;
    std::string   unsupportedReason;  // set when kind == Unsupported
};

/// Resource counts in the form SDL_GPU wants them.
struct ResourceCounts
{
    std::uint32_t samplers                 = 0;
    std::uint32_t readOnlyStorageTextures  = 0;
    std::uint32_t readOnlyStorageBuffers   = 0;
    std::uint32_t readWriteStorageTextures = 0;
    std::uint32_t readWriteStorageBuffers  = 0;
    std::uint32_t uniformBuffers           = 0;
};

struct ShaderReflection
{
    ShaderStage                  stage = ShaderStage::Vertex;
    std::string                  entryPoint;
    std::array<std::uint32_t, 3> localSize{1, 1, 1};  // compute workgroup size
    std::vector<ResourceBinding> resources;

    [[nodiscard]] ResourceCounts counts() const;
};

/// Converts a .spv file's bytes into 32-bit words (little-endian host; the SPIR-V magic is
/// checked).
[[nodiscard]] std::expected<std::vector<std::uint32_t>, std::string> spirvWordsFromBytes(
    std::span<const std::byte> bytes);

/// Reflects the module's single entry point. Fails on malformed SPIR-V or multiple entry points.
[[nodiscard]] std::expected<ShaderReflection, std::string> reflectSpirv(
    std::span<const std::uint32_t> words);

/// Checks the descriptor-set layout SDL_GPU requires on Vulkan. Returns one message per problem.
///   vertex:   set 0 = sampled textures, storage textures, storage buffers; set 1 = uniform buffers
///   fragment: set 2 = sampled textures, storage textures, storage buffers; set 3 = uniform buffers
///   compute:  set 0 = sampled textures, read-only storage textures, read-only storage buffers;
///             set 1 = read-write storage textures, read-write storage buffers; set 2 = uniform
///             buffers
/// Within a set, bindings are numbered 0, 1, 2, ... in exactly that order of kinds.
[[nodiscard]] std::vector<std::string> validateSdlGpuLayout(const ShaderReflection& reflection);

}  // namespace StarshipSimulator::gpu
