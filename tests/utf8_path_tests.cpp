#include "StarshipSimulator/core/utf8_path.h"

#include <array>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

TEST(Utf8Path, KeepsNamesOutsideAscii)
{
    // Tsiolkovsky in Cyrillic, and an accented Latin name: no single Windows code page holds both,
    // so path::string() would mangle one of them there.
    const std::array<std::string, 2> names = {
        "\xD0\xA6\xD0\xB8\xD0\xBE\xD0\xBB\xD0\xBA\xD0\xBE\xD0\xB2\xD1\x81\xD0\xBA\xD0\xB8\xD0\xB9."
        "toml",
        "Ger\xC3\xA1rd.toml"};
    for (const std::string& name : names)
    {
        const std::filesystem::path path = pathFromUtf8("habitats/" + name);
        EXPECT_EQ(utf8String(path.filename()), name);
        // The real characters, not the bytes read as some code page. (EXPECT_TRUE: a GoogleTest
        // built as C++17 cannot print a u8string.)
        EXPECT_TRUE(path.filename().u8string() == std::u8string(name.begin(), name.end()));
    }
}

TEST(Utf8Path, JoinsLikeAnyOtherPath)
{
    const std::filesystem::path joined = pathFromUtf8("data") / "presets";
    EXPECT_TRUE(joined.generic_u8string() == u8"data/presets");
    EXPECT_EQ(utf8String(pathFromUtf8("")), "");
}

}  // namespace
