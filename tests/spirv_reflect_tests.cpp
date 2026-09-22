#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/utf8_path.h"

namespace
{

namespace gpu = StarshipSimulator::gpu;

// SPIR-V opcodes and enums used to hand-assemble small test modules.
constexpr std::uint32_t kOpName           = 5;
constexpr std::uint32_t kOpEntryPoint     = 15;
constexpr std::uint32_t kOpExecutionMode  = 16;
constexpr std::uint32_t kOpTypeFloat      = 22;
constexpr std::uint32_t kOpTypeVector     = 23;
constexpr std::uint32_t kOpTypeImage      = 25;
constexpr std::uint32_t kOpTypeSampler    = 26;
constexpr std::uint32_t kOpTypeSampled    = 27;
constexpr std::uint32_t kOpTypeStruct     = 30;
constexpr std::uint32_t kOpTypePointer    = 32;
constexpr std::uint32_t kOpVariable       = 59;
constexpr std::uint32_t kOpDecorate       = 71;
constexpr std::uint32_t kOpMemberDecorate = 72;

constexpr std::uint32_t kFragment  = 4;
constexpr std::uint32_t kGLCompute = 5;

constexpr std::uint32_t kUniformConstant = 0;
constexpr std::uint32_t kUniform         = 2;
constexpr std::uint32_t kPushConstant    = 9;

constexpr std::uint32_t kBlock         = 2;
constexpr std::uint32_t kBufferBlock   = 3;
constexpr std::uint32_t kNonWritable   = 24;
constexpr std::uint32_t kBinding       = 33;
constexpr std::uint32_t kDescriptorSet = 34;

/// Assembles SPIR-V words. Ids are handed out by the caller; the bound is fixed at 200.
class SpirvBuilder
{
public:
    SpirvBuilder() : words_{0x07230203U, 0x00010000U, 0U, 200U, 0U} { }

    void op(std::uint32_t opcode, std::initializer_list<std::uint32_t> operands)
    {
        words_.push_back((static_cast<std::uint32_t>(operands.size() + 1) << 16U) | opcode);
        words_.insert(words_.end(), operands.begin(), operands.end());
    }

    /// An instruction whose last operand is a literal string (OpName, OpEntryPoint).
    void opWithString(std::uint32_t opcode, std::initializer_list<std::uint32_t> operands,
                      std::string_view text)
    {
        std::vector<std::uint32_t> packed((text.size() / 4) + 1, 0U);
        for (std::size_t i = 0; i < text.size(); ++i)
        {
            packed[i / 4] |= static_cast<std::uint32_t>(static_cast<unsigned char>(text[i]))
                             << (8U * static_cast<std::uint32_t>(i % 4));
        }
        words_.push_back((static_cast<std::uint32_t>(operands.size() + packed.size() + 1) << 16U) |
                         opcode);
        words_.insert(words_.end(), operands.begin(), operands.end());
        words_.insert(words_.end(), packed.begin(), packed.end());
    }

    void decorateBinding(std::uint32_t id, std::uint32_t set, std::uint32_t binding)
    {
        op(kOpDecorate, {id, kDescriptorSet, set});
        op(kOpDecorate, {id, kBinding, binding});
    }

    [[nodiscard]] const std::vector<std::uint32_t>& words() const { return words_; }

private:
    std::vector<std::uint32_t> words_;
};

// Shared type ids.
constexpr std::uint32_t kMain            = 1;
constexpr std::uint32_t kFloat           = 10;
constexpr std::uint32_t kVec4            = 11;
constexpr std::uint32_t kImage2D         = 12;
constexpr std::uint32_t kSampled2D       = 13;
constexpr std::uint32_t kWritableImage2D = 14;
constexpr std::uint32_t kUboStruct       = 15;
constexpr std::uint32_t kSsboStruct      = 16;
constexpr std::uint32_t kSampler         = 17;

void addTypes(SpirvBuilder& b)
{
    b.op(kOpTypeFloat, {kFloat, 32});
    b.op(kOpTypeVector, {kVec4, kFloat, 4});
    b.op(kOpTypeImage, {kImage2D, kFloat, 1, 0, 0, 0, 1, 0});  // sampled image
    b.op(kOpTypeSampled, {kSampled2D, kImage2D});
    b.op(kOpTypeImage, {kWritableImage2D, kFloat, 1, 0, 0, 0, 2, 1});  // storage image, rgba32f
    b.op(kOpTypeStruct, {kUboStruct, kVec4});
    b.op(kOpDecorate, {kUboStruct, kBlock});
    b.op(kOpTypeStruct, {kSsboStruct, kVec4});
    b.op(kOpDecorate, {kSsboStruct, kBufferBlock});  // SPIR-V 1.0 storage buffer
    b.op(kOpTypeSampler, {kSampler});
}

/// Declares a variable of the given pointee type and storage class, named and bound.
void addVariable(SpirvBuilder& b, std::uint32_t id, std::uint32_t pointee, std::uint32_t storage,
                 std::string_view name, std::uint32_t set, std::uint32_t binding)
{
    const std::uint32_t pointer = id + 100;
    b.op(kOpTypePointer, {pointer, storage, pointee});
    b.op(kOpVariable, {pointer, id, storage});
    b.opWithString(kOpName, {id}, name);
    b.decorateBinding(id, set, binding);
}

SpirvBuilder fragmentShader()
{
    SpirvBuilder b;
    b.opWithString(kOpEntryPoint, {kFragment, kMain}, "main");
    addTypes(b);
    return b;
}

gpu::ShaderReflection reflect(const SpirvBuilder& b)
{
    auto reflection = gpu::reflectSpirv(b.words());
    EXPECT_TRUE(reflection.has_value()) << reflection.error();
    return reflection.value_or(gpu::ShaderReflection{});
}

bool mentions(const std::vector<std::string>& problems, std::string_view text)
{
    return std::ranges::any_of(problems,
                               [&](const std::string& problem) { return problem.contains(text); });
}

TEST(SpirvReflect, FragmentShaderWithTextureAndUniformBlockFollowsSdlLayout)
{
    SpirvBuilder b = fragmentShader();
    addVariable(b, 20, kSampled2D, kUniformConstant, "albedo", 2, 0);
    addVariable(b, 21, kSampled2D, kUniformConstant, "detail", 2, 1);
    addVariable(b, 22, kUboStruct, kUniform, "params", 3, 0);

    const gpu::ShaderReflection reflection = reflect(b);
    EXPECT_EQ(reflection.stage, gpu::ShaderStage::Fragment);
    EXPECT_EQ(reflection.entryPoint, "main");
    ASSERT_EQ(reflection.resources.size(), 3U);
    EXPECT_EQ(reflection.resources[0].name, "albedo");
    EXPECT_EQ(reflection.resources[0].kind, gpu::ResourceKind::SampledTexture);
    EXPECT_EQ(reflection.resources[2].kind, gpu::ResourceKind::UniformBuffer);

    const gpu::ResourceCounts counts = reflection.counts();
    EXPECT_EQ(counts.samplers, 2U);
    EXPECT_EQ(counts.uniformBuffers, 1U);
    EXPECT_TRUE(gpu::validateSdlGpuLayout(reflection).empty());
}

TEST(SpirvReflect, TexturesInTheVertexSetOfAFragmentShaderAreRejected)
{
    SpirvBuilder b = fragmentShader();
    addVariable(b, 20, kSampled2D, kUniformConstant, "albedo", 0, 0);  // vertex-stage set
    EXPECT_TRUE(mentions(gpu::validateSdlGpuLayout(reflect(b)), "must be in set 2"));
}

TEST(SpirvReflect, StorageBuffersMustFollowTexturesWithoutGaps)
{
    SpirvBuilder b = fragmentShader();
    addVariable(b, 20, kSsboStruct, kUniform, "lights", 2, 0);  // should be binding 1
    b.op(kOpMemberDecorate, {kSsboStruct, 0, kNonWritable});
    addVariable(b, 21, kSampled2D, kUniformConstant, "albedo", 2, 1);  // should be binding 0
    const auto problems = gpu::validateSdlGpuLayout(reflect(b));
    EXPECT_TRUE(mentions(problems, "lights: binding 0"));
    EXPECT_TRUE(mentions(problems, "albedo: binding 1"));
}

TEST(SpirvReflect, WritableStorageInAGraphicsShaderIsRejected)
{
    SpirvBuilder b = fragmentShader();
    addVariable(b, 20, kSsboStruct, kUniform, "particles", 2, 0);  // no NonWritable
    const gpu::ShaderReflection reflection = reflect(b);
    EXPECT_FALSE(reflection.resources.front().readOnly);
    EXPECT_TRUE(mentions(gpu::validateSdlGpuLayout(reflection), "only allowed in compute"));
}

TEST(SpirvReflect, ComputeShaderLayoutAndWorkgroupSize)
{
    SpirvBuilder b;
    b.opWithString(kOpEntryPoint, {kGLCompute, kMain}, "main");
    b.op(kOpExecutionMode, {kMain, 17, 8, 8, 1});  // LocalSize 8 8 1
    addTypes(b);
    addVariable(b, 20, kSampled2D, kUniformConstant, "input", 0, 0);
    addVariable(b, 21, kWritableImage2D, kUniformConstant, "output", 1, 0);
    addVariable(b, 22, kUboStruct, kUniform, "params", 2, 0);

    const gpu::ShaderReflection reflection = reflect(b);
    EXPECT_EQ(reflection.stage, gpu::ShaderStage::Compute);
    EXPECT_EQ(reflection.localSize[0], 8U);
    EXPECT_EQ(reflection.localSize[1], 8U);
    EXPECT_EQ(reflection.localSize[2], 1U);
    const gpu::ResourceCounts counts = reflection.counts();
    EXPECT_EQ(counts.samplers, 1U);
    EXPECT_EQ(counts.readWriteStorageTextures, 1U);
    EXPECT_EQ(counts.uniformBuffers, 1U);
    EXPECT_TRUE(gpu::validateSdlGpuLayout(reflection).empty());
}

TEST(SpirvReflect, UnsupportedResourcesAreExplained)
{
    SpirvBuilder b = fragmentShader();
    addVariable(b, 20, kSampler, kUniformConstant, "linearSampler", 2, 0);
    addVariable(b, 21, kImage2D, kUniformConstant, "texture", 2, 1);
    const std::uint32_t pushPointer = 150;
    b.op(kOpTypePointer, {pushPointer, kPushConstant, kUboStruct});
    b.op(kOpVariable, {pushPointer, 22, kPushConstant});
    b.opWithString(kOpName, {22}, "push");

    const auto problems = gpu::validateSdlGpuLayout(reflect(b));
    EXPECT_TRUE(mentions(problems, "linearSampler: separate samplers"));
    EXPECT_TRUE(mentions(problems, "texture: separate texture objects"));
    EXPECT_TRUE(mentions(problems, "push: push constants"));
}

TEST(SpirvReflect, RejectsMalformedModules)
{
    EXPECT_FALSE(gpu::reflectSpirv(std::vector<std::uint32_t>{1, 2, 3}).has_value());

    const SpirvBuilder         truncated = fragmentShader();
    std::vector<std::uint32_t> words     = truncated.words();
    words.push_back((10U << 16U) | kOpDecorate);  // claims 10 words, has 1
    EXPECT_FALSE(gpu::reflectSpirv(words).has_value());

    SpirvBuilder noEntry;
    addTypes(noEntry);
    EXPECT_FALSE(gpu::reflectSpirv(noEntry.words()).has_value());

    const std::vector<std::byte> oddSize(21, std::byte{0});
    EXPECT_FALSE(gpu::spirvWordsFromBytes(oddSize).has_value());
}

// Every shader the build produced must be valid for SDL_GPU.
TEST(SpirvReflect, AllBuiltShadersFollowTheSdlGpuLayout)
{
    const std::filesystem::path directory =
        StarshipSimulator::pathFromUtf8(STARSHIPSIMULATOR_SHADER_DIR);
    ASSERT_TRUE(std::filesystem::is_directory(directory)) << directory;
    int checked = 0;
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
        for (const std::string& problem : gpu::validateSdlGpuLayout(*reflection))
        {
            ADD_FAILURE() << entry.path().filename() << ": " << problem;
        }
        ++checked;
    }
    EXPECT_GE(checked, 5);
}

TEST(SpirvReflect, TonemapShaderHasTwoTexturesAndOneUniformBlock)
{
    const std::filesystem::path path =
        StarshipSimulator::pathFromUtf8(STARSHIPSIMULATOR_SHADER_DIR) / "tonemap.frag.spv";
    std::ifstream                file(path, std::ios::binary);
    const std::string            contents((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());
    const std::vector<std::byte> bytes(std::as_bytes(std::span(contents)).begin(),
                                       std::as_bytes(std::span(contents)).end());
    const auto reflection = gpu::reflectSpirv(gpu::spirvWordsFromBytes(bytes).value());
    ASSERT_TRUE(reflection.has_value());
    EXPECT_EQ(reflection->stage, gpu::ShaderStage::Fragment);
    EXPECT_EQ(reflection->counts().samplers, 2U);  // the scene and the colour grade
    EXPECT_EQ(reflection->counts().uniformBuffers, 1U);
}

}  // namespace
