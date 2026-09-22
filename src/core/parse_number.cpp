#include "StarshipSimulator/core/parse_number.h"

#include <charconv>
#include <memory>
#include <optional>
#include <string_view>
#include <system_error>

#ifndef __cpp_lib_to_chars  // parseDouble's fallback
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <string>
#endif

namespace StarshipSimulator
{

std::optional<int> parseInt(std::string_view text)
{
    int               value = 0;
    const char* const first = std::to_address(text.begin());
    const char* const last  = std::to_address(text.end());
    const auto [end, error] = std::from_chars(first, last, value);
    if (text.empty() || error != std::errc{} || end != last)
    {
        return std::nullopt;
    }
    return value;
}

std::optional<double> parseDouble(std::string_view text)
{
    if (text.empty())
    {
        return std::nullopt;
    }
#ifdef __cpp_lib_to_chars
    double            value = 0.0;
    const char* const first = std::to_address(text.begin());
    const char* const last  = std::to_address(text.end());
    const auto [end, error] = std::from_chars(first, last, value);
    if (error != std::errc{} || end != last)
    {
        return std::nullopt;
    }
    return value;
#else
    // libc++ before LLVM 20 (Apple's, for one) has no floating-point std::from_chars. strtod reads
    // the C locale's decimal point, which stays '.' because nothing here calls setlocale. It skips
    // leading spaces and takes a '+', which from_chars would not, so those are refused first.
    const auto lead = static_cast<unsigned char>(text.front());
    if (std::isspace(lead) != 0 || lead == '+')
    {
        return std::nullopt;
    }
    const std::string terminated(text);
    char*             end = nullptr;
    errno                 = 0;
    const double value    = std::strtod(terminated.c_str(), &end);
    if (errno == ERANGE || end != std::to_address(terminated.end()))
    {
        return std::nullopt;
    }
    return value;
#endif
}

}  // namespace StarshipSimulator
