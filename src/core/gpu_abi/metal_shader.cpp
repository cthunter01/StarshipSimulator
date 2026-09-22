#include "StarshipSimulator/core/gpu_abi/metal_shader.h"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <spirv.hpp>
#include <spirv_msl.hpp>

#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"

namespace StarshipSimulator::gpu
{

namespace
{

/// The shader's resources of one kind, in binding order.
std::vector<const ResourceBinding*> ofKind(const ShaderReflection& reflection, ResourceKind kind)
{
    std::vector<const ResourceBinding*> found;
    for (const ResourceBinding& resource : reflection.resources)
    {
        if (resource.kind == kind)
        {
            found.push_back(&resource);
        }
    }
    std::ranges::sort(found, {}, [](const ResourceBinding* resource) { return resource->binding; });
    return found;
}

std::uint32_t count(const std::vector<const ResourceBinding*>& resources)
{
    return static_cast<std::uint32_t>(resources.size());
}

}  // namespace

std::vector<MetalBinding> metalBindings(const ShaderReflection& reflection)
{
    const auto sampled  = ofKind(reflection, ResourceKind::SAMPLED_TEXTURE);
    const auto images   = ofKind(reflection, ResourceKind::STORAGE_TEXTURE);
    const auto uniforms = ofKind(reflection, ResourceKind::UNIFORM_BUFFER);
    const auto storage  = ofKind(reflection, ResourceKind::STORAGE_BUFFER);

    std::vector<MetalBinding> bindings;
    const auto                place = [&](const ResourceBinding& resource) -> MetalBinding& {
        return bindings.emplace_back(
            MetalBinding{.set = resource.set, .binding = resource.binding});
    };
    for (std::uint32_t i = 0; i < count(sampled); ++i)
    {
        MetalBinding& binding = place(*sampled[i]);
        binding.texture       = i;
        binding.sampler       = i;
    }
    for (std::uint32_t i = 0; i < count(images); ++i)
    {
        place(*images[i]).texture = count(sampled) + i;
    }
    for (std::uint32_t i = 0; i < count(uniforms); ++i)
    {
        place(*uniforms[i]).buffer = i;
    }
    for (std::uint32_t i = 0; i < count(storage); ++i)
    {
        place(*storage[i]).buffer = count(uniforms) + i;
    }
    return bindings;
}

std::expected<MetalShader, std::string> translateToMetal(std::span<const std::uint32_t> spirv,
                                                         const ShaderReflection&        reflection)
{
    spv::ExecutionModel model = spv::ExecutionModelVertex;
    switch (reflection.stage)
    {
        case ShaderStage::VERTEX:
            model = spv::ExecutionModelVertex;
            break;
        case ShaderStage::FRAGMENT:
            model = spv::ExecutionModelFragment;
            break;
        case ShaderStage::COMPUTE:
            return std::unexpected(std::string("compute shaders are not translated to Metal yet"));
    }
    try
    {
        spirv_cross::CompilerMSL          compiler(spirv.data(), spirv.size());
        spirv_cross::CompilerMSL::Options options = compiler.get_msl_options();
        options.platform                          = spirv_cross::CompilerMSL::Options::macOS;
        options.set_msl_version(2, 1);
        compiler.set_msl_options(options);
        for (const MetalBinding& metal : metalBindings(reflection))
        {
            spirv_cross::MSLResourceBinding binding;
            binding.stage       = model;
            binding.desc_set    = metal.set;
            binding.binding     = metal.binding;
            binding.count       = 1;
            binding.msl_buffer  = metal.buffer;
            binding.msl_texture = metal.texture;
            binding.msl_sampler = metal.sampler;
            compiler.add_msl_resource_binding(binding);
        }
        MetalShader shader;
        shader.source     = compiler.compile();
        shader.entryPoint = compiler.get_cleansed_entry_point_name(reflection.entryPoint, model);
        return shader;
    }
    catch (const std::exception& error)
    {
        return std::unexpected(std::format("SPIR-V to MSL: {}", error.what()));
    }
}

}  // namespace StarshipSimulator::gpu
