#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace StarshipSimulator::gpu
{

namespace
{

// Values from the SPIR-V specification (unified1).
namespace spv
{
constexpr std::uint32_t kMagic      = 0x07230203U;
constexpr std::size_t   kHeaderSize = 5;  // words
constexpr std::uint32_t kMaxIdBound = 1U << 22U;

constexpr std::uint32_t OpName             = 5;
constexpr std::uint32_t OpEntryPoint       = 15;
constexpr std::uint32_t OpExecutionMode    = 16;
constexpr std::uint32_t OpTypeImage        = 25;
constexpr std::uint32_t OpTypeSampler      = 26;
constexpr std::uint32_t OpTypeSampledImage = 27;
constexpr std::uint32_t OpTypeArray        = 28;
constexpr std::uint32_t OpTypeRuntimeArray = 29;
constexpr std::uint32_t OpTypeStruct       = 30;
constexpr std::uint32_t OpTypePointer      = 32;
constexpr std::uint32_t OpVariable         = 59;
constexpr std::uint32_t OpDecorate         = 71;
constexpr std::uint32_t OpMemberDecorate   = 72;

constexpr std::uint32_t DecorationBlock         = 2;
constexpr std::uint32_t DecorationBufferBlock   = 3;
constexpr std::uint32_t DecorationNonWritable   = 24;
constexpr std::uint32_t DecorationBinding       = 33;
constexpr std::uint32_t DecorationDescriptorSet = 34;

constexpr std::uint32_t StorageClassUniformConstant = 0;
constexpr std::uint32_t StorageClassUniform         = 2;
constexpr std::uint32_t StorageClassPushConstant    = 9;
constexpr std::uint32_t StorageClassStorageBuffer   = 12;

constexpr std::uint32_t ExecutionModelVertex    = 0;
constexpr std::uint32_t ExecutionModelFragment  = 4;
constexpr std::uint32_t ExecutionModelGLCompute = 5;
constexpr std::uint32_t ExecutionModeLocalSize  = 17;

constexpr std::uint32_t ImageSampledWithSampler = 1;
constexpr std::uint32_t ImageStorage            = 2;
}  // namespace spv

/// Everything we learn about one SPIR-V id.
struct IdInfo
{
    std::uint32_t                opcode = 0;  // the instruction that defines it (types, variables)
    std::vector<std::uint32_t>   operands;    // all operands of that instruction
    std::string                  name;
    std::optional<std::uint32_t> set;
    std::optional<std::uint32_t> binding;
    bool                         block              = false;
    bool                         bufferBlock        = false;
    bool                         nonWritable        = false;
    std::uint32_t                nonWritableMembers = 0;
};

struct Module
{
    std::vector<IdInfo>        ids;
    std::vector<std::uint32_t> variables;
    std::uint32_t              entryPointCount = 0;
    ShaderReflection           reflection;

    [[nodiscard]] const IdInfo* find(std::uint32_t id) const
    {
        return id < ids.size() ? &ids[id] : nullptr;
    }
};

/// Decodes a nul-terminated literal string packed four characters per word.
std::string decodeString(std::span<const std::uint32_t> words)
{
    std::string text;
    for (const std::uint32_t word : words)
    {
        for (std::uint32_t shift = 0; shift < 32; shift += 8)
        {
            const auto character = static_cast<char>((word >> shift) & 0xFFU);
            if (character == '\0')
            {
                return text;
            }
            text.push_back(character);
        }
    }
    return text;
}

std::expected<ShaderStage, std::string> stageFromModel(std::uint32_t model)
{
    switch (model)
    {
        case spv::ExecutionModelVertex:
            return ShaderStage::Vertex;
        case spv::ExecutionModelFragment:
            return ShaderStage::Fragment;
        case spv::ExecutionModelGLCompute:
            return ShaderStage::Compute;
        default:
            return std::unexpected(std::format("unsupported execution model {}", model));
    }
}

void applyDecoration(IdInfo& info, std::uint32_t decoration,
                     std::span<const std::uint32_t> literals)
{
    switch (decoration)
    {
        case spv::DecorationBlock:
            info.block = true;
            break;
        case spv::DecorationBufferBlock:
            info.bufferBlock = true;
            break;
        case spv::DecorationNonWritable:
            info.nonWritable = true;
            break;
        case spv::DecorationBinding:
            if (!literals.empty())
            {
                info.binding = literals[0];
            }
            break;
        case spv::DecorationDescriptorSet:
            if (!literals.empty())
            {
                info.set = literals[0];
            }
            break;
        default:
            break;
    }
}

std::expected<void, std::string> readEntryPoint(Module&                        module,
                                                std::span<const std::uint32_t> operands)
{
    // OpEntryPoint: execution model, function id, name, interface ids...
    if (operands.size() < 3)
    {
        return std::unexpected("truncated OpEntryPoint");
    }
    auto stage = stageFromModel(operands[0]);
    if (!stage)
    {
        return std::unexpected(stage.error());
    }
    module.reflection.stage      = *stage;
    module.reflection.entryPoint = decodeString(operands.subspan(2));
    ++module.entryPointCount;
    return {};
}

bool isDefinition(std::uint32_t opcode)
{
    switch (opcode)
    {
        case spv::OpTypeImage:
        case spv::OpTypeSampler:
        case spv::OpTypeSampledImage:
        case spv::OpTypeArray:
        case spv::OpTypeRuntimeArray:
        case spv::OpTypeStruct:
        case spv::OpTypePointer:
        case spv::OpVariable:
            return true;
        default:
            return false;
    }
}

/// Records one instruction. Returns an error message for malformed input.
std::expected<void, std::string> readInstruction(Module& module, std::uint32_t opcode,
                                                 std::span<const std::uint32_t> operands)
{
    // The id an instruction is about: operand 1 for variables (after the result type), else operand
    // 0.
    const std::size_t idIndex = opcode == spv::OpVariable ? 1 : 0;
    const bool        about   = opcode == spv::OpName || opcode == spv::OpDecorate ||
                                opcode == spv::OpMemberDecorate || isDefinition(opcode);
    if (!about)
    {
        return opcode == spv::OpEntryPoint ? readEntryPoint(module, operands)
                                           : std::expected<void, std::string>{};
    }
    if (idIndex >= operands.size() || operands[idIndex] >= module.ids.size())
    {
        return std::unexpected(std::format("opcode {} refers to an invalid id", opcode));
    }
    IdInfo& info = module.ids[operands[idIndex]];

    if (opcode == spv::OpName)
    {
        info.name = decodeString(operands.subspan(1));
    }
    else if (opcode == spv::OpDecorate && operands.size() >= 2)
    {
        applyDecoration(info, operands[1], operands.subspan(2));
    }
    else if (opcode == spv::OpMemberDecorate && operands.size() >= 3 &&
             operands[2] == spv::DecorationNonWritable)
    {
        ++info.nonWritableMembers;
    }
    else if (isDefinition(opcode))
    {
        info.opcode = opcode;
        info.operands.assign(operands.begin(), operands.end());
        if (opcode == spv::OpVariable)
        {
            module.variables.push_back(operands[idIndex]);
        }
    }
    return {};
}

void readExecutionMode(Module& module, std::uint32_t opcode,
                       std::span<const std::uint32_t> operands)
{
    if (opcode == spv::OpExecutionMode && operands.size() >= 5 &&
        operands[1] == spv::ExecutionModeLocalSize)
    {
        module.reflection.localSize = {operands[2], operands[3], operands[4]};
    }
}

/// Fills kind/readOnly/unsupportedReason from the variable's pointee type.
void classifyType(const IdInfo& type, const IdInfo& variable, std::uint32_t storageClass,
                  ResourceBinding& resource)
{
    switch (type.opcode)
    {
        case spv::OpTypeSampledImage:
            resource.kind = ResourceKind::SampledTexture;
            return;
        case spv::OpTypeImage:
        {
            // OpTypeImage operands: result, sampled type, dim, depth, arrayed, MS, sampled, format
            const std::uint32_t sampled = type.operands.size() > 6 ? type.operands[6] : 0;
            if (sampled == spv::ImageStorage)
            {
                resource.kind     = ResourceKind::StorageTexture;
                resource.readOnly = variable.nonWritable;
                return;
            }
            resource.unsupportedReason =
                sampled == spv::ImageSampledWithSampler
                    ? "separate texture objects are not supported; use a combined sampler2D"
                    : "unknown image usage";
            return;
        }
        case spv::OpTypeSampler:
            resource.unsupportedReason =
                "separate samplers are not supported; use a combined sampler2D";
            return;
        case spv::OpTypeArray:
        case spv::OpTypeRuntimeArray:
            resource.unsupportedReason = "arrays of resources are not supported by SDL_GPU";
            return;
        case spv::OpTypeStruct:
            break;
        default:
            resource.unsupportedReason = "unsupported resource type";
            return;
    }

    // Blocks. SPIR-V 1.0 (Vulkan 1.0) marks storage buffers as Uniform + BufferBlock.
    const bool storage = (storageClass == spv::StorageClassUniform && type.bufferBlock) ||
                         (storageClass == spv::StorageClassStorageBuffer && type.block);
    if (storage)
    {
        const std::size_t memberCount = type.operands.size() - 1;  // operands after the result id
        resource.kind                 = ResourceKind::StorageBuffer;
        resource.readOnly =
            variable.nonWritable || (memberCount > 0 && type.nonWritableMembers >= memberCount);
    }
    else if (storageClass == spv::StorageClassUniform && type.block)
    {
        resource.kind = ResourceKind::UniformBuffer;
    }
    else
    {
        resource.unsupportedReason = "unknown block type";
    }
}

bool isResourceStorageClass(std::uint32_t storageClass)
{
    return storageClass == spv::StorageClassUniformConstant ||
           storageClass == spv::StorageClassUniform ||
           storageClass == spv::StorageClassStorageBuffer ||
           storageClass == spv::StorageClassPushConstant;
}

/// Classifies a variable. Returns nullopt for variables that are not shader resources.
std::optional<ResourceBinding> classifyVariable(const Module& module, const IdInfo& variable)
{
    // OpVariable operands: result type (a pointer), result id, storage class.
    if (variable.operands.size() < 3 || !isResourceStorageClass(variable.operands[2]))
    {
        return std::nullopt;
    }
    const std::uint32_t storageClass = variable.operands[2];

    ResourceBinding resource;
    resource.name    = variable.name;
    resource.set     = variable.set.value_or(0);
    resource.binding = variable.binding.value_or(0);

    // OpTypePointer operands: result, storage class, pointee type.
    const IdInfo* pointer = module.find(variable.operands[0]);
    const IdInfo* type    = (pointer != nullptr && pointer->operands.size() >= 3)
                                ? module.find(pointer->operands[2])
                                : nullptr;
    if (type == nullptr)
    {
        resource.unsupportedReason = "unknown type";
    }
    else if (storageClass == spv::StorageClassPushConstant)
    {
        resource.unsupportedReason =
            "push constants are not supported by SDL_GPU; use a uniform block";
    }
    else if (!variable.set || !variable.binding)
    {
        resource.unsupportedReason = "missing layout(set = ..., binding = ...)";
    }
    else
    {
        classifyType(*type, variable, storageClass, resource);
    }
    if (resource.name.empty() && type != nullptr)
    {
        resource.name = type->name;  // anonymous blocks are named after their type
    }
    if (resource.name.empty())  // optimized shaders may have no names at all
    {
        resource.name =
            std::format("resource at set {} binding {}", resource.set, resource.binding);
    }
    return resource;
}

// ---- Layout validation -------------------------------------------------------------------------

enum class Category : std::uint8_t
{
    Sampled,
    ReadOnlyStorageTexture,
    ReadOnlyStorageBuffer,
    ReadWriteStorageTexture,
    ReadWriteStorageBuffer,
    Uniform,
};

Category categoryOf(const ResourceBinding& resource)
{
    switch (resource.kind)
    {
        case ResourceKind::SampledTexture:
            return Category::Sampled;
        case ResourceKind::StorageTexture:
            return resource.readOnly ? Category::ReadOnlyStorageTexture
                                     : Category::ReadWriteStorageTexture;
        case ResourceKind::StorageBuffer:
            return resource.readOnly ? Category::ReadOnlyStorageBuffer
                                     : Category::ReadWriteStorageBuffer;
        case ResourceKind::UniformBuffer:
        case ResourceKind::Unsupported:
            break;
    }
    return Category::Uniform;
}

std::string_view categoryName(Category category)
{
    switch (category)
    {
        case Category::Sampled:
            return "sampled texture";
        case Category::ReadOnlyStorageTexture:
            return "read-only storage texture";
        case Category::ReadOnlyStorageBuffer:
            return "read-only storage buffer";
        case Category::ReadWriteStorageTexture:
            return "read-write storage texture";
        case Category::ReadWriteStorageBuffer:
            return "read-write storage buffer";
        case Category::Uniform:
            return "uniform buffer";
    }
    return "resource";
}

/// Where SDL_GPU expects a category: its descriptor set and its rank among the kinds in that set.
struct Slot
{
    std::uint32_t set  = 0;
    std::uint32_t rank = 0;
};

std::optional<Slot> expectedSlot(ShaderStage stage, Category category)
{
    const bool          compute     = stage == ShaderStage::Compute;
    const std::uint32_t resourceSet = stage == ShaderStage::Fragment ? 2 : 0;
    switch (category)
    {
        case Category::Sampled:
            return Slot{.set = resourceSet, .rank = 0};
        case Category::ReadOnlyStorageTexture:
            return Slot{.set = resourceSet, .rank = 1};
        case Category::ReadOnlyStorageBuffer:
            return Slot{.set = resourceSet, .rank = 2};
        case Category::ReadWriteStorageTexture:
            return compute ? std::optional(Slot{.set = 1, .rank = 0}) : std::nullopt;
        case Category::ReadWriteStorageBuffer:
            return compute ? std::optional(Slot{.set = 1, .rank = 1}) : std::nullopt;
        case Category::Uniform:
            return Slot{.set = compute ? 2U : resourceSet + 1, .rank = 0};
    }
    return std::nullopt;
}

struct Placed
{
    const ResourceBinding* resource = nullptr;
    Slot                   slot;
};

/// Puts each resource in its expected slot, reporting unsupported or misplaced ones.
std::vector<Placed> placeResources(const ShaderReflection&   reflection,
                                   std::vector<std::string>& problems)
{
    std::vector<Placed> placed;
    for (const ResourceBinding& resource : reflection.resources)
    {
        if (resource.kind == ResourceKind::Unsupported)
        {
            problems.push_back(std::format("{}: {}", resource.name, resource.unsupportedReason));
            continue;
        }
        const Category category = categoryOf(resource);
        const auto     slot     = expectedSlot(reflection.stage, category);
        if (!slot)
        {
            problems.push_back(
                std::format("{}: {}s are only allowed in compute shaders (declare it readonly)",
                            resource.name, categoryName(category)));
        }
        else if (resource.set != slot->set)
        {
            problems.push_back(std::format("{}: a {} must be in set {}, not set {}", resource.name,
                                           categoryName(category), slot->set, resource.set));
        }
        else
        {
            placed.push_back({.resource = &resource, .slot = *slot});
        }
    }
    return placed;
}

/// Within a set, kinds come in rank order and bindings count up from 0 without gaps.
void checkBindingRanges(std::span<const Placed> placed, std::vector<std::string>& problems)
{
    for (const Placed& item : placed)
    {
        std::uint32_t first = 0;  // first binding for this kind = number of lower-ranked resources
        std::uint32_t count = 0;  // resources of this kind in this set
        for (const Placed& other : placed)
        {
            if (other.slot.set == item.slot.set)
            {
                first += other.slot.rank < item.slot.rank ? 1U : 0U;
                count += other.slot.rank == item.slot.rank ? 1U : 0U;
            }
        }
        const std::uint32_t binding = item.resource->binding;
        if (binding < first || binding >= first + count)
        {
            problems.push_back(
                std::format("{}: binding {} in set {}; SDL_GPU expects its {}s at bindings {}..{} "
                            "(order: sampled "
                            "textures, storage textures, storage buffers)",
                            item.resource->name, binding, item.slot.set,
                            categoryName(categoryOf(*item.resource)), first, first + count - 1));
        }
    }
}

void checkDuplicates(std::span<const Placed> placed, std::vector<std::string>& problems)
{
    for (std::size_t i = 0; i < placed.size(); ++i)
    {
        for (std::size_t j = i + 1; j < placed.size(); ++j)
        {
            const ResourceBinding& a = *placed[i].resource;
            const ResourceBinding& b = *placed[j].resource;
            if (a.set == b.set && a.binding == b.binding)
            {
                problems.push_back(std::format("{} and {} share set {} binding {}", a.name, b.name,
                                               a.set, a.binding));
            }
        }
    }
}

void checkLimits(const ShaderReflection& reflection, std::vector<std::string>& problems)
{
    // Per-stage limits from SDL_sysgpu.h.
    struct Limit
    {
        std::uint32_t    count;
        std::uint32_t    max;
        std::string_view what;
    };
    const ResourceCounts       counts = reflection.counts();
    const std::array<Limit, 6> limits{{
        {.count = counts.samplers, .max = 16, .what = "sampled textures"},
        {.count = counts.readOnlyStorageTextures, .max = 8, .what = "read-only storage textures"},
        {.count = counts.readOnlyStorageBuffers, .max = 8, .what = "read-only storage buffers"},
        {.count = counts.readWriteStorageTextures, .max = 8, .what = "read-write storage textures"},
        {.count = counts.readWriteStorageBuffers, .max = 8, .what = "read-write storage buffers"},
        {.count = counts.uniformBuffers, .max = 4, .what = "uniform buffers"},
    }};
    for (const Limit& limit : limits)
    {
        if (limit.count > limit.max)
        {
            problems.push_back(std::format("{} {} exceed SDL_GPU's limit of {}", limit.count,
                                           limit.what, limit.max));
        }
    }
}

}  // namespace

ResourceCounts ShaderReflection::counts() const
{
    ResourceCounts counts;
    for (const ResourceBinding& resource : resources)
    {
        switch (resource.kind)
        {
            case ResourceKind::SampledTexture:
                ++counts.samplers;
                break;
            case ResourceKind::StorageTexture:
                ++(resource.readOnly ? counts.readOnlyStorageTextures
                                     : counts.readWriteStorageTextures);
                break;
            case ResourceKind::StorageBuffer:
                ++(resource.readOnly ? counts.readOnlyStorageBuffers
                                     : counts.readWriteStorageBuffers);
                break;
            case ResourceKind::UniformBuffer:
                ++counts.uniformBuffers;
                break;
            case ResourceKind::Unsupported:
                break;
        }
    }
    return counts;
}

std::expected<std::vector<std::uint32_t>, std::string> spirvWordsFromBytes(
    std::span<const std::byte> bytes)
{
    if (bytes.size() < spv::kHeaderSize * sizeof(std::uint32_t) ||
        bytes.size() % sizeof(std::uint32_t) != 0)
    {
        return std::unexpected(std::format("not a SPIR-V module ({} bytes)", bytes.size()));
    }
    std::vector<std::uint32_t> words(bytes.size() / sizeof(std::uint32_t));
    std::memcpy(words.data(), bytes.data(), bytes.size());
    if (words.front() != spv::kMagic)
    {
        return std::unexpected("not a SPIR-V module (bad magic number)");
    }
    return words;
}

std::expected<ShaderReflection, std::string> reflectSpirv(std::span<const std::uint32_t> words)
{
    if (words.size() < spv::kHeaderSize || words[0] != spv::kMagic)
    {
        return std::unexpected("not a SPIR-V module");
    }
    const std::uint32_t bound = words[3];
    if (bound == 0 || bound > spv::kMaxIdBound)
    {
        return std::unexpected(std::format("implausible SPIR-V id bound {}", bound));
    }

    Module module;
    module.ids.resize(bound);
    for (std::size_t offset = spv::kHeaderSize; offset < words.size();)
    {
        const std::uint32_t wordCount = words[offset] >> 16U;
        const std::uint32_t opcode    = words[offset] & 0xFFFFU;
        if (wordCount == 0 || offset + wordCount > words.size())
        {
            return std::unexpected(std::format("truncated SPIR-V instruction at word {}", offset));
        }
        const auto operands = words.subspan(offset + 1, wordCount - 1);
        if (auto read = readInstruction(module, opcode, operands); !read)
        {
            return std::unexpected(read.error());
        }
        readExecutionMode(module, opcode, operands);
        offset += wordCount;
    }

    if (module.entryPointCount != 1)
    {
        return std::unexpected(
            std::format("expected exactly one entry point, found {}", module.entryPointCount));
    }

    for (const std::uint32_t variableId : module.variables)
    {
        if (auto resource = classifyVariable(module, module.ids[variableId]); resource)
        {
            module.reflection.resources.push_back(std::move(*resource));
        }
    }
    std::ranges::sort(module.reflection.resources, [](const auto& a, const auto& b) {
        return a.set != b.set ? a.set < b.set : a.binding < b.binding;
    });
    return std::move(module.reflection);
}

std::vector<std::string> validateSdlGpuLayout(const ShaderReflection& reflection)
{
    std::vector<std::string>  problems;
    const std::vector<Placed> placed = placeResources(reflection, problems);
    checkBindingRanges(placed, problems);
    checkDuplicates(placed, problems);
    checkLimits(reflection, problems);
    return problems;
}

}  // namespace StarshipSimulator::gpu
