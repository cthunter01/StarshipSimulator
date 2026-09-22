#include "StarshipSimulator/core/assets/assets.h"

#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/utf8_path.h"

namespace StarshipSimulator::assets
{

namespace
{

struct StbFree
{
    void operator()(void* memory) const noexcept { stbi_image_free(memory); }
};

/// Little-endian reader over a byte span that never reads past the end.
class Reader
{
public:
    explicit Reader(std::span<const std::byte> bytes) : bytes_(bytes) { }

    [[nodiscard]] bool        ok() const { return ok_; }
    [[nodiscard]] std::size_t offset() const { return offset_; }
    void                      seek(std::size_t offset)
    {
        ok_     = ok_ && offset <= bytes_.size();
        offset_ = std::min(offset, bytes_.size());
    }

    template <typename T>
    T read()
    {
        T value{};
        if (!ok_ || bytes_.size() - offset_ < sizeof(T))
        {
            ok_ = false;
            return value;
        }
        std::memcpy(&value, bytes_.subspan(offset_, sizeof(T)).data(), sizeof(T));
        offset_ += sizeof(T);
        return value;
    }

    std::string readString()  // nul-terminated
    {
        std::string text;
        while (ok_ && offset_ < bytes_.size())
        {
            const auto c = static_cast<char>(bytes_[offset_++]);
            if (c == '\0')
            {
                return text;
            }
            text.push_back(c);
        }
        ok_ = false;
        return text;
    }

    std::span<const std::byte> take(std::size_t size)
    {
        if (!ok_ || bytes_.size() - offset_ < size)
        {
            ok_ = false;
            return {};
        }
        const auto result = bytes_.subspan(offset_, size);
        offset_ += size;
        return result;
    }

private:
    std::span<const std::byte> bytes_;
    std::size_t                offset_ = 0;
    bool                       ok_     = true;
};

// ---- OpenEXR -----------------------------------------------------------------------------------

constexpr std::uint32_t kExrMagic        = 20000630;
constexpr int           kPixelTypeHalf   = 1;
constexpr int           kPixelTypeFloat  = 2;
constexpr std::uint8_t  kCompressionNone = 0;
constexpr std::uint8_t  kCompressionZips = 2;
constexpr std::uint8_t  kCompressionZip  = 3;
constexpr std::uint16_t kHalfOne         = 0x3C00;

struct ExrChannel
{
    std::string name;
    int         pixelType = kPixelTypeHalf;
    int         target    = -1;  // RGBA index, -1 = ignored
};

struct ExrHeader
{
    std::vector<ExrChannel> channels;
    std::uint8_t            compression = kCompressionNone;
    int                     xMin        = 0;
    int                     yMin        = 0;
    int                     xMax        = -1;
    int                     yMax        = -1;
};

int channelTarget(std::string_view name)
{
    constexpr std::array<std::string_view, 4> kNames{"R", "G", "B", "A"};
    for (std::size_t i = 0; i < kNames.size(); ++i)
    {
        if (name == kNames.at(i))
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::expected<ExrHeader, std::string> readExrHeader(Reader& reader)
{
    if (reader.read<std::uint32_t>() != kExrMagic)
    {
        return std::unexpected("not an OpenEXR file");
    }
    const auto version = reader.read<std::uint32_t>();
    if ((version & 0xFFU) != 2 || (version & 0x1E00U) != 0)  // tiled, long names, deep, multipart
    {
        return std::unexpected("only single-part scanline OpenEXR images are supported");
    }
    ExrHeader header;
    while (reader.ok())
    {
        const std::string name = reader.readString();
        if (name.empty())
        {
            break;
        }
        reader.readString();  // attribute type name
        const auto size = reader.read<std::int32_t>();
        Reader     value(reader.take(static_cast<std::size_t>(std::max(size, 0))));
        if (name == "channels")
        {
            while (value.ok())
            {
                const std::string channel = value.readString();
                if (channel.empty())
                {
                    break;
                }
                const auto pixelType = value.read<std::int32_t>();
                value.take(8);               // pLinear, reserved, xSampling
                value.read<std::int32_t>();  // ySampling
                header.channels.push_back(
                    {.name = channel, .pixelType = pixelType, .target = channelTarget(channel)});
            }
        }
        else if (name == "compression")
        {
            header.compression = value.read<std::uint8_t>();
        }
        else if (name == "dataWindow")
        {
            header.xMin = value.read<std::int32_t>();
            header.yMin = value.read<std::int32_t>();
            header.xMax = value.read<std::int32_t>();
            header.yMax = value.read<std::int32_t>();
        }
    }
    if (!reader.ok() || header.channels.empty() || header.xMax < header.xMin ||
        header.yMax < header.yMin)
    {
        return std::unexpected("damaged OpenEXR header");
    }
    for (const ExrChannel& channel : header.channels)
    {
        if (channel.pixelType != kPixelTypeHalf && channel.pixelType != kPixelTypeFloat)
        {
            return std::unexpected("only HALF and FLOAT OpenEXR channels are supported");
        }
    }
    if (header.compression != kCompressionNone && header.compression != kCompressionZips &&
        header.compression != kCompressionZip)
    {
        return std::unexpected(
            std::format("unsupported OpenEXR compression {}", header.compression));
    }
    return header;
}

/// Undoes OpenEXR's ZIP pre-processing: a byte delta predictor, then split even/odd byte halves.
std::vector<std::byte> unpredictZip(std::span<const std::byte> data)
{
    std::vector<std::uint8_t> t(data.size());
    std::memcpy(t.data(), data.data(), data.size());
    for (std::size_t i = 1; i < t.size(); ++i)
    {
        t[i] = static_cast<std::uint8_t>(t[i - 1] + t[i] - 128U);
    }
    std::vector<std::byte> out(data.size());
    const std::size_t      half = (data.size() + 1) / 2;
    for (std::size_t i = 0; i < out.size(); ++i)
    {
        out[i] = static_cast<std::byte>((i % 2 == 0) ? t[i / 2] : t[half + (i / 2)]);
    }
    return out;
}

/// Writes one block of decoded scanlines into the image.
void storeScanlines(const ExrHeader& header, std::span<const std::byte> block, int firstLine,
                    int lineCount, HalfImage& image)
{
    const int   width = image.width;
    std::size_t at    = 0;
    for (int line = 0; line < lineCount; ++line)
    {
        const int y = firstLine + line;
        for (const ExrChannel& channel : header.channels)
        {
            const std::size_t sampleSize = channel.pixelType == kPixelTypeHalf ? 2 : 4;
            for (int x = 0; x < width; ++x, at += sampleSize)
            {
                if (channel.target < 0 || y < 0 || y >= image.height)
                {
                    continue;
                }
                std::uint16_t half = 0;
                if (channel.pixelType == kPixelTypeHalf)
                {
                    std::memcpy(&half, block.subspan(at, 2).data(), 2);
                }
                else
                {
                    float value = 0.0F;
                    std::memcpy(&value, block.subspan(at, 4).data(), 4);
                    half =
                        static_cast<std::uint16_t>(glm::packHalf2x16(Vec2f(value, 0.0F)) & 0xFFFFU);
                }
                const auto pixel = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) +
                                   static_cast<std::size_t>(x);
                image.rgba[(pixel * 4) + static_cast<std::size_t>(channel.target)] = half;
            }
        }
    }
}

}  // namespace

std::expected<std::vector<std::byte>, std::string> readFile(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        return std::unexpected(std::format("cannot open {}", utf8String(path)));
    }
    const std::vector<char> chars((std::istreambuf_iterator<char>(file)),
                                  std::istreambuf_iterator<char>());
    std::vector<std::byte>  bytes(chars.size());
    std::memcpy(bytes.data(), chars.data(), chars.size());
    return bytes;
}

std::expected<std::vector<std::byte>, std::string> inflateZlib(
    std::span<const std::byte> compressed, std::size_t decompressedSize)
{
    if (std::cmp_greater(compressed.size(), std::numeric_limits<int>::max()) ||
        std::cmp_greater(decompressedSize, std::numeric_limits<int>::max()))
    {
        return std::unexpected("zlib stream too large");
    }
    std::vector<char> in(compressed.size());
    std::memcpy(in.data(), compressed.data(), compressed.size());
    std::vector<char> buffer(decompressedSize);
    const int written = stbi_zlib_decode_buffer(buffer.data(), static_cast<int>(buffer.size()),
                                                in.data(), static_cast<int>(in.size()));
    if (std::cmp_not_equal(written, decompressedSize))
    {
        return std::unexpected("damaged zlib data");
    }
    std::vector<std::byte> out(decompressedSize);
    std::memcpy(out.data(), buffer.data(), buffer.size());
    return out;
}

std::expected<std::vector<std::byte>, std::string> gunzip(std::span<const std::byte> compressed)
{
    // RFC 1952: 10-byte header, optional fields, raw deflate data, CRC32 and size trailer.
    Reader     reader(compressed);
    const auto magic  = reader.read<std::uint16_t>();
    const auto method = reader.read<std::uint8_t>();
    const auto flags  = reader.read<std::uint8_t>();
    reader.take(6);  // mtime, xfl, os
    if (!reader.ok() || magic != 0x8B1F || method != 8)
    {
        return std::unexpected("not a gzip file");
    }
    if ((flags & 0x04U) != 0)  // FEXTRA
    {
        reader.take(reader.read<std::uint16_t>());
    }
    if ((flags & 0x08U) != 0)  // FNAME
    {
        reader.readString();
    }
    if ((flags & 0x10U) != 0)  // FCOMMENT
    {
        reader.readString();
    }
    if ((flags & 0x02U) != 0)  // FHCRC
    {
        reader.take(2);
    }
    if (!reader.ok() || compressed.size() < reader.offset() + 8)
    {
        return std::unexpected("damaged gzip header");
    }
    const auto deflate =
        compressed.subspan(reader.offset(), compressed.size() - reader.offset() - 8);
    if (std::cmp_greater(deflate.size(), std::numeric_limits<int>::max()))
    {
        return std::unexpected("gzip file too large");
    }
    std::vector<char> in(deflate.size());
    std::memcpy(in.data(), deflate.data(), deflate.size());
    int                                  length = 0;
    const std::unique_ptr<char, StbFree> out(
        stbi_zlib_decode_noheader_malloc(in.data(), static_cast<int>(in.size()), &length));
    if (!out || length < 0)
    {
        return std::unexpected("damaged gzip data");
    }
    std::vector<std::byte> bytes(static_cast<std::size_t>(length));
    std::memcpy(bytes.data(), out.get(), bytes.size());
    return bytes;
}

std::expected<HalfImage, std::string> decodeExr(std::span<const std::byte> bytes)
{
    Reader     reader(bytes);
    const auto header = readExrHeader(reader);
    if (!header)
    {
        return std::unexpected(header.error());
    }
    HalfImage image;
    image.width  = header->xMax - header->xMin + 1;
    image.height = header->yMax - header->yMin + 1;
    image.rgba.assign(
        static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4, 0);
    for (std::size_t i = 3; i < image.rgba.size(); i += 4)
    {
        image.rgba[i] = kHalfOne;  // opaque unless there is an A channel
    }

    std::size_t bytesPerLine = 0;
    for (const ExrChannel& channel : header->channels)
    {
        bytesPerLine +=
            static_cast<std::size_t>(image.width) * (channel.pixelType == kPixelTypeHalf ? 2U : 4U);
    }
    const int                  linesPerBlock = header->compression == kCompressionZip ? 16 : 1;
    const int                  blocks        = (image.height + linesPerBlock - 1) / linesPerBlock;
    std::vector<std::uint64_t> offsets(static_cast<std::size_t>(blocks));
    for (std::uint64_t& offset : offsets)
    {
        offset = reader.read<std::uint64_t>();
    }
    if (!reader.ok())
    {
        return std::unexpected("damaged OpenEXR offset table");
    }

    for (const std::uint64_t offset : offsets)
    {
        reader.seek(static_cast<std::size_t>(offset));
        const int  y          = reader.read<std::int32_t>() - header->yMin;
        const auto packedSize = static_cast<std::size_t>(std::max(reader.read<std::int32_t>(), 0));
        const auto packed     = reader.take(packedSize);
        if (!reader.ok() || y < 0 || y >= image.height)
        {
            return std::unexpected("damaged OpenEXR data");
        }
        const int         lines = std::min(linesPerBlock, image.height - y);
        const std::size_t size  = bytesPerLine * static_cast<std::size_t>(lines);
        if (header->compression == kCompressionNone || packedSize == size)
        {
            storeScanlines(*header, packed, y, lines, image);  // stored raw
            continue;
        }
        const auto inflated = inflateZlib(packed, size);
        if (!inflated)
        {
            return std::unexpected(inflated.error());
        }
        storeScanlines(*header, unpredictZip(*inflated), y, lines, image);
    }
    return image;
}

std::expected<Image8, std::string> decodeImage(std::span<const std::byte> bytes)
{
    if (std::cmp_greater(bytes.size(), std::numeric_limits<int>::max()))
    {
        return std::unexpected("image file too large");
    }
    std::vector<stbi_uc> data(bytes.size());
    std::memcpy(data.data(), bytes.data(), bytes.size());
    int                                     width    = 0;
    int                                     height   = 0;
    int                                     channels = 0;
    const std::unique_ptr<stbi_uc, StbFree> pixels(stbi_load_from_memory(
        data.data(), static_cast<int>(data.size()), &width, &height, &channels, 4));
    if (!pixels)
    {
        return std::unexpected(std::format("cannot decode image: {}", stbi_failure_reason()));
    }
    Image8 image;
    image.width  = width;
    image.height = height;
    image.rgba.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
    std::memcpy(image.rgba.data(), pixels.get(), image.rgba.size());
    return image;
}

}  // namespace StarshipSimulator::assets
