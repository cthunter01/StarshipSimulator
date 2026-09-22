#include "StarshipSimulator/core/utf8_path.h"

#include <filesystem>
#include <string>
#include <string_view>

namespace StarshipSimulator
{

std::filesystem::path pathFromUtf8(std::string_view text)
{
    return {std::u8string(text.begin(), text.end())};
}

std::string utf8String(const std::filesystem::path& path)
{
    const std::u8string text = path.u8string();
    return {text.begin(), text.end()};
}

}  // namespace StarshipSimulator
