#include "StarshipSimulator/core/assets/assets.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/utf8_path.h"

// stb_image_write's zlib compressor, used to build test data. It is defined (not static) in the
// stb library core links, but only declared inside stb_image_write.h's implementation section.
// NOLINTNEXTLINE(readability-identifier-naming): stb's own name
extern "C" unsigned char* stbi_zlib_compress(unsigned char* data, int dataLength, int* outLength,
                                             int quality);

namespace StarshipSimulator::assets
{
namespace
{

struct FreeDeleter
{
    void operator()(unsigned char* memory) const noexcept
    {
        // Allocated by stb with malloc.
        std::free(memory);  // NOLINT(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
    }
};

std::vector<std::byte> bytesOf(std::string_view text)
{
    std::vector<std::byte> bytes(text.size());
    std::memcpy(bytes.data(), text.data(), text.size());
    return bytes;
}

/// A zlib stream (2-byte header, deflate data, Adler-32).
std::vector<std::byte> zlibCompress(std::span<const std::byte> data)
{
    std::vector<unsigned char> input(data.size());
    std::memcpy(input.data(), data.data(), data.size());
    int                                               length = 0;
    const std::unique_ptr<unsigned char, FreeDeleter> compressed(
        stbi_zlib_compress(input.data(), static_cast<int>(input.size()), &length, 8));
    std::vector<std::byte> out(static_cast<std::size_t>(length));
    std::memcpy(out.data(), compressed.get(), out.size());
    return out;
}

template <typename T>
void append(std::vector<std::byte>& out, T value)
{
    const auto at = out.size();
    out.resize(at + sizeof(T));
    std::memcpy(&out[at], &value, sizeof(T));
}

void appendText(std::vector<std::byte>& out, std::string_view text)
{
    for (const char c : text)
    {
        out.push_back(static_cast<std::byte>(c));
    }
    out.push_back(std::byte{0});
}

/// A .gz file: header, raw deflate data (the zlib stream without header and checksum), trailer.
std::vector<std::byte> gzip(std::span<const std::byte> data, bool withFileName)
{
    const std::vector<std::byte> zlib = zlibCompress(data);
    std::vector<std::byte>       out;
    append<std::uint16_t>(out, 0x8B1F);
    append<std::uint8_t>(out, 8);                        // deflate
    append<std::uint8_t>(out, withFileName ? 0x08 : 0);  // FNAME
    append<std::uint32_t>(out, 0);                       // mtime
    append<std::uint8_t>(out, 0);                        // xfl
    append<std::uint8_t>(out, 3);                        // Unix
    if (withFileName)
    {
        appendText(out, "stars.csv");
    }
    out.insert(out.end(), zlib.begin() + 2, zlib.end() - 4);
    append<std::uint32_t>(out, 0);  // CRC-32 (not checked)
    append<std::uint32_t>(out, static_cast<std::uint32_t>(data.size()));
    return out;
}

TEST(Assets, InflatesZlib)
{
    const std::string            text(5000, 'x');
    const std::vector<std::byte> compressed = zlibCompress(bytesOf(text));
    EXPECT_LT(compressed.size(), 200U);
    const auto inflated = inflateZlib(compressed, text.size());
    ASSERT_TRUE(inflated.has_value());
    EXPECT_EQ(inflated.value(), bytesOf(text));
    EXPECT_FALSE(inflateZlib(compressed, text.size() + 1).has_value());  // wrong size
}

TEST(Assets, Gunzips)
{
    std::string text;
    for (int i = 0; i < 2000; ++i)
    {
        text += std::to_string(i * 7919 % 1000) + ",";
    }
    for (const bool withFileName : {false, true})
    {
        const auto result = gunzip(gzip(bytesOf(text), withFileName));
        ASSERT_TRUE(result.has_value());
        EXPECT_EQ(result.value(), bytesOf(text));
    }
    EXPECT_FALSE(gunzip(bytesOf("not a gzip file at all")).has_value());
    EXPECT_FALSE(gunzip({}).has_value());
}

// ---- OpenEXR ------------------------------------------------------------------------------------

/// OpenEXR's ZIP pre-processing (the inverse of what the decoder undoes): split even and odd
/// bytes into two halves, then delta-encode.
std::vector<std::byte> predictZip(std::span<const std::byte> data)
{
    std::vector<std::uint8_t> t(data.size());
    const std::size_t         half = (data.size() + 1) / 2;
    for (std::size_t i = 0; i < data.size(); ++i)
    {
        t[(i % 2 == 0) ? i / 2 : half + (i / 2)] = static_cast<std::uint8_t>(data[i]);
    }
    std::vector<std::byte> out(data.size());
    for (std::size_t i = 0; i < t.size(); ++i)
    {
        const int previous = i == 0 ? 0 : t[i - 1];
        const int delta    = i == 0 ? t[0] : t[i] - previous + 128 + 256;
        out[i]             = static_cast<std::byte>(delta & 0xFF);
    }
    return out;
}

void appendAttribute(std::vector<std::byte>& out, std::string_view name, std::string_view type,
                     const std::vector<std::byte>& value)
{
    appendText(out, name);
    appendText(out, type);
    append<std::int32_t>(out, static_cast<std::int32_t>(value.size()));
    out.insert(out.end(), value.begin(), value.end());
}

std::uint16_t half(float value)
{
    return static_cast<std::uint16_t>(glm::packHalf2x16(Vec2f(value, 0.0F)) & 0xFFFFU);
}

/// The test image's value for a channel (0 = B, 1 = G, 2 = R, as EXR sorts them) at a pixel.
float sample(int channel, int x, int y)
{
    return (static_cast<float>(channel + 1) * 0.25F) + (static_cast<float>(x) * 0.5F) +
           (static_cast<float>(y) * 8.0F);
}

/// A width x height scanline image with B, G, R half channels in the given compression.
std::vector<std::byte> makeExr(int width, int height, std::uint8_t compression)
{
    std::vector<std::byte> out;
    append<std::uint32_t>(out, 20000630);
    append<std::uint32_t>(out, 2);

    std::vector<std::byte> channels;
    for (const std::string_view name : {"B", "G", "R"})
    {
        appendText(channels, name);
        append<std::int32_t>(channels, 1);   // HALF
        append<std::uint32_t>(channels, 0);  // pLinear + reserved
        append<std::int32_t>(channels, 1);
        append<std::int32_t>(channels, 1);
    }
    channels.push_back(std::byte{0});
    appendAttribute(out, "channels", "chlist", channels);
    appendAttribute(out, "compression", "compression", {std::byte{compression}});
    std::vector<std::byte> box;
    for (const int v : {0, 0, width - 1, height - 1})
    {
        append<std::int32_t>(box, v);
    }
    appendAttribute(out, "dataWindow", "box2i", box);
    appendAttribute(out, "displayWindow", "box2i", box);
    appendAttribute(out, "lineOrder", "lineOrder", {std::byte{0}});
    out.push_back(std::byte{0});  // end of header

    const int         linesPerBlock = compression == 3 ? 16 : 1;
    const int         blocks        = (height + linesPerBlock - 1) / linesPerBlock;
    const std::size_t tableAt       = out.size();
    out.resize(out.size() + (static_cast<std::size_t>(blocks) * 8));
    for (int block = 0; block < blocks; ++block)
    {
        const std::uint64_t offset = out.size();
        std::memcpy(&out[tableAt + (static_cast<std::size_t>(block) * 8)], &offset, 8);
        std::vector<std::byte> raw;
        const int              first = block * linesPerBlock;
        for (int y = first; y < std::min(height, first + linesPerBlock); ++y)
        {
            for (int channel = 0; channel < 3; ++channel)
            {
                for (int x = 0; x < width; ++x)
                {
                    append<std::uint16_t>(raw, half(sample(channel, x, y)));
                }
            }
        }
        const std::vector<std::byte> packed =
            compression == 0 ? raw : zlibCompress(predictZip(raw));
        append<std::int32_t>(out, first);
        append<std::int32_t>(out, static_cast<std::int32_t>(packed.size()));
        out.insert(out.end(), packed.begin(), packed.end());
    }
    return out;
}

void expectTestImage(const HalfImage& image, int width, int height)
{
    ASSERT_EQ(image.width, width);
    ASSERT_EQ(image.height, height);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const auto pixel = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)) +
                               static_cast<std::size_t>(x);
            // RGBA order: R is EXR channel 2, G 1, B 0.
            EXPECT_EQ(image.rgba[(pixel * 4) + 0], half(sample(2, x, y)));
            EXPECT_EQ(image.rgba[(pixel * 4) + 1], half(sample(1, x, y)));
            EXPECT_EQ(image.rgba[(pixel * 4) + 2], half(sample(0, x, y)));
            EXPECT_EQ(image.rgba[(pixel * 4) + 3], half(1.0F));
        }
    }
}

TEST(Assets, DecodesZipCompressedExr)
{
    const auto image = decodeExr(makeExr(24, 37, 3));  // 37 lines: a partial last block
    ASSERT_TRUE(image.has_value()) << image.error();
    expectTestImage(image.value(), 24, 37);
}

TEST(Assets, DecodesUncompressedExr)
{
    const auto image = decodeExr(makeExr(5, 3, 0));
    ASSERT_TRUE(image.has_value()) << image.error();
    expectTestImage(image.value(), 5, 3);
}

TEST(Assets, RejectsDamagedExr)
{
    EXPECT_FALSE(decodeExr(bytesOf("not an image")).has_value());
    std::vector<std::byte> truncated = makeExr(8, 20, 3);
    truncated.resize(truncated.size() - 10);
    EXPECT_FALSE(decodeExr(truncated).has_value());
    std::vector<std::byte> tiled = makeExr(4, 4, 3);
    tiled[5]                     = std::byte{0x02};  // version flags: the "tiled" bit (0x200)
    EXPECT_FALSE(decodeExr(tiled).has_value());
}

TEST(Assets, RejectsUndecodableImages)
{
    EXPECT_FALSE(decodeImage(bytesOf("definitely not a JPEG")).has_value());
}

// ---- The real sky data, when it has been downloaded ---------------------------------------------

std::filesystem::path skyData(std::string_view name)
{
    return pathFromUtf8(STARSHIPSIMULATOR_SKY_DATA_DIR) / name;
}

TEST(SkyData, MilkyWayMapDecodes)
{
    const auto path = skyData("milkyway_2020_4k.exr");
    if (!std::filesystem::exists(path))
    {
        GTEST_SKIP() << "sky data not downloaded";
    }
    const auto bytes = readFile(path);
    ASSERT_TRUE(bytes.has_value());
    const auto image = decodeExr(bytes.value());
    ASSERT_TRUE(image.has_value()) << image.error();
    EXPECT_EQ(image->width, 4096);
    EXPECT_EQ(image->height, 2048);

    // The galactic centre (RA 17h45.6m, Dec -28.94) is far brighter than the galactic pole
    // (RA 12h51.4m, Dec +27.13). The map has RA 0 at the centre, increasing to the left.
    const auto luminance = [&](double raHours, double decDegrees) {
        const double u     = 0.5 - (raHours / 24.0);
        const double v     = 0.5 - (decDegrees / 180.0);
        const auto   x     = static_cast<std::size_t>((u - std::floor(u)) * image->width);
        const auto   y     = static_cast<std::size_t>(v * image->height);
        const auto   pixel = (y * static_cast<std::size_t>(image->width)) + x;
        float        sum   = 0.0F;
        for (std::size_t c = 0; c < 3; ++c)
        {
            sum += glm::unpackHalf2x16(image->rgba[(pixel * 4) + c]).x;
        }
        return sum;
    };
    EXPECT_GT(luminance(17.76, -28.94), 5.0F * luminance(12.857, 27.13));
}

TEST(SkyData, PlanetTexturesDecode)
{
    for (const std::string_view name :
         {"earth_day_2048.jpg", "earth_night_3600.jpg", "moon_1k.jpg"})
    {
        const auto path = skyData(name);
        if (!std::filesystem::exists(path))
        {
            GTEST_SKIP() << "sky data not downloaded";
        }
        const auto image = decodeImage(readFile(path).value());
        ASSERT_TRUE(image.has_value()) << name;
        EXPECT_EQ(image->width, 2 * image->height) << name;  // equirectangular
        EXPECT_EQ(image->rgba.size(), static_cast<std::size_t>(image->width) *
                                          static_cast<std::size_t>(image->height) * 4);
    }
}

}  // namespace
}  // namespace StarshipSimulator::assets
