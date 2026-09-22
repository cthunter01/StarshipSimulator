#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace StarshipSimulator
{

/// Paths to and from UTF-8, the text of SDL, ImGui, command-line arguments (SDL hands them over in
/// UTF-8 on Windows too) and our messages. On Windows std::filesystem::path(const char*) and
/// path::string() use the ANSI code page instead, which mangles any name outside it; on Linux and
/// macOS these change nothing.
[[nodiscard]] std::filesystem::path pathFromUtf8(std::string_view text);
[[nodiscard]] std::string           utf8String(const std::filesystem::path& path);

}  // namespace StarshipSimulator
