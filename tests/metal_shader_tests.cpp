#include "StarshipSimulator/core/gpu_abi/metal_shader.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <regex>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"
#include "StarshipSimulator/core/utf8_path.h"

namespace
{

namespace gpu = StarshipSimulator::gpu;

gpu::ResourceBinding resource(gpu::ResourceKind kind, std::uint32_t set, std::uint32_t binding)
{
    return {.name              = "r" + std::to_string(binding),
            .kind              = kind,
            .readOnly          = true,
            .set               = set,
            .binding           = binding,
            .unsupportedReason = {}};
}

const gpu::MetalBinding* find(const std::vector<gpu::MetalBinding>& bindings, std::uint32_t set,
                              std::uint32_t binding)
{
    for (const gpu::MetalBinding& metal : bindings)
    {
        if (metal.set == set && metal.binding == binding)
        {
            return &metal;
        }
    }
    return nullptr;
}

/// The indices a kind of Metal attribute ([[buffer(n)]] etc.) is given in some MSL source, sorted.
std::vector<std::uint32_t> indicesOf(const std::string& source, const std::string& attribute)
{
    const std::regex           pattern(R"(\[\[)" + attribute + R"(\((\d+)\)\]\])");
    std::vector<std::uint32_t> indices;
    for (auto match = std::sregex_iterator(source.begin(), source.end(), pattern);
         match != std::sregex_iterator(); ++match)
    {
        indices.push_back(static_cast<std::uint32_t>(std::stoul((*match)[1].str())));
    }
    std::ranges::sort(indices);
    return indices;
}

TEST(MetalShader, NumbersEachKindOfResourceTheWaySdlGpuBindsThemOnMetal)
{
    // A vertex shader with everything: set 0 holds two sampled textures, a storage texture and two
    // storage buffers (in that order), set 1 two uniform blocks.
    gpu::ShaderReflection vertex;
    vertex.stage     = gpu::ShaderStage::VERTEX;
    vertex.resources = {resource(gpu::ResourceKind::STORAGE_BUFFER, 0, 4),
                        resource(gpu::ResourceKind::UNIFORM_BUFFER, 1, 1),
                        resource(gpu::ResourceKind::SAMPLED_TEXTURE, 0, 0),
                        resource(gpu::ResourceKind::STORAGE_TEXTURE, 0, 2),
                        resource(gpu::ResourceKind::STORAGE_BUFFER, 0, 3),
                        resource(gpu::ResourceKind::UNIFORM_BUFFER, 1, 0),
                        resource(gpu::ResourceKind::SAMPLED_TEXTURE, 0, 1)};
    const std::vector<gpu::MetalBinding> bindings = gpu::metalBindings(vertex);
    ASSERT_EQ(bindings.size(), vertex.resources.size());

    // Textures: sampled first, then storage; each sampled texture's sampler has its index.
    EXPECT_EQ(find(bindings, 0, 0)->texture, 0U);
    EXPECT_EQ(find(bindings, 0, 0)->sampler, 0U);
    EXPECT_EQ(find(bindings, 0, 1)->texture, 1U);
    EXPECT_EQ(find(bindings, 0, 1)->sampler, 1U);
    EXPECT_EQ(find(bindings, 0, 2)->texture, 2U);
    // Buffers: uniform blocks first, then the storage buffers after them.
    EXPECT_EQ(find(bindings, 1, 0)->buffer, 0U);
    EXPECT_EQ(find(bindings, 1, 1)->buffer, 1U);
    EXPECT_EQ(find(bindings, 0, 3)->buffer, 2U);
    EXPECT_EQ(find(bindings, 0, 4)->buffer, 3U);
}

TEST(MetalShader, RefusesComputeShaders)
{
    gpu::ShaderReflection compute;
    compute.stage      = gpu::ShaderStage::COMPUTE;
    compute.entryPoint = "main";
    EXPECT_FALSE(gpu::translateToMetal({}, compute).has_value());
}

// Every shader the build produced must translate, and its resources must land where SDL_GPU binds
// them on Metal: [[buffer]] below 14, where the vertex buffers start, and no index beyond the
// shader's own resources.
TEST(MetalShader, AllBuiltShadersTranslate)
{
    const std::filesystem::path directory =
        StarshipSimulator::pathFromUtf8(STARSHIPSIMULATOR_SHADER_DIR);
    ASSERT_TRUE(std::filesystem::is_directory(directory)) << directory;
    int translated = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.path().extension() != ".spv")
        {
            continue;
        }
        std::ifstream                file(entry.path(), std::ios::binary);
        const std::string            contents((std::istreambuf_iterator<char>(file)),
                                              std::istreambuf_iterator<char>());
        const std::vector<std::byte> bytes(std::as_bytes(std::span(contents)).begin(),
                                           std::as_bytes(std::span(contents)).end());
        const auto                   words = gpu::spirvWordsFromBytes(bytes);
        ASSERT_TRUE(words.has_value()) << entry.path() << ": " << words.error();
        const auto reflection = gpu::reflectSpirv(*words);
        ASSERT_TRUE(reflection.has_value()) << entry.path() << ": " << reflection.error();

        const auto metal = gpu::translateToMetal(*words, *reflection);
        ASSERT_TRUE(metal.has_value()) << entry.path().filename() << ": " << metal.error();
        EXPECT_EQ(metal->entryPoint, "main0") << entry.path().filename();
        EXPECT_NE(metal->source.find(" main0("), std::string::npos) << entry.path().filename();

        const gpu::ResourceCounts counts = reflection->counts();
        for (const std::uint32_t index : indicesOf(metal->source, "buffer"))
        {
            EXPECT_LT(index, counts.uniformBuffers + counts.readOnlyStorageBuffers)
                << entry.path().filename();
            EXPECT_LT(index, 14U) << entry.path().filename();
        }
        for (const std::uint32_t index : indicesOf(metal->source, "texture"))
        {
            EXPECT_LT(index, counts.samplers + counts.readOnlyStorageTextures)
                << entry.path().filename();
        }
        for (const std::uint32_t index : indicesOf(metal->source, "sampler"))
        {
            EXPECT_LT(index, counts.samplers) << entry.path().filename();
        }
        ++translated;
    }
    EXPECT_GE(translated, 30);
}

TEST(MetalShader, TonemapReadsItsTwoTexturesAndItsUniformsWhereSdlPutsThem)
{
    const std::filesystem::path path =
        StarshipSimulator::pathFromUtf8(STARSHIPSIMULATOR_SHADER_DIR) / "tonemap.frag.spv";
    std::ifstream                file(path, std::ios::binary);
    const std::string            contents((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());
    const std::vector<std::byte> bytes(std::as_bytes(std::span(contents)).begin(),
                                       std::as_bytes(std::span(contents)).end());
    const auto                   words      = gpu::spirvWordsFromBytes(bytes).value();
    const auto                   reflection = gpu::reflectSpirv(words).value();
    const auto                   metal      = gpu::translateToMetal(words, reflection);
    ASSERT_TRUE(metal.has_value()) << metal.error();
    EXPECT_NE(metal->source.find("fragment "), std::string::npos);
    EXPECT_EQ(indicesOf(metal->source, "texture"), (std::vector<std::uint32_t>{0, 1}));
    EXPECT_EQ(indicesOf(metal->source, "sampler"), (std::vector<std::uint32_t>{0, 1}));
    EXPECT_EQ(indicesOf(metal->source, "buffer"), (std::vector<std::uint32_t>{0}));
}

}  // namespace
