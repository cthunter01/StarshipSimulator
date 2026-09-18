#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

// Loading and decoding of data files: raw bytes, zlib/gzip, OpenEXR (for the Milky Way map) and
// JPEG/PNG images (for planet textures).
namespace StarshipSimulator::assets
{

[[nodiscard]] std::expected<std::vector<std::byte>, std::string> readFile(
    const std::filesystem::path& path);

/// Inflates a zlib stream (with its 2-byte header) of known decompressed size.
[[nodiscard]] std::expected<std::vector<std::byte>, std::string> inflateZlib(
    std::span<const std::byte> compressed, std::size_t decompressedSize);

/// Decompresses a .gz file's contents (a single gzip member).
[[nodiscard]] std::expected<std::vector<std::byte>, std::string> gunzip(
    std::span<const std::byte> compressed);

/// An image with four 16-bit half-float channels per pixel (RGBA), rows from the top.
struct HalfImage
{
    int                        width  = 0;
    int                        height = 0;
    std::vector<std::uint16_t> rgba;
};

/// Decodes a single-part scanline OpenEXR image with HALF or FLOAT R, G, B (and optional A)
/// channels and NONE, ZIPS or ZIP compression (what NASA's sky maps use).
[[nodiscard]] std::expected<HalfImage, std::string> decodeExr(std::span<const std::byte> bytes);

/// An 8-bit RGBA image, rows from the top.
struct Image8
{
    int                       width  = 0;
    int                       height = 0;
    std::vector<std::uint8_t> rgba;
};

/// Decodes a JPEG or PNG image.
[[nodiscard]] std::expected<Image8, std::string> decodeImage(std::span<const std::byte> bytes);

}  // namespace StarshipSimulator::assets
