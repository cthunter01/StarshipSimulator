#pragma once

#include <optional>
#include <string_view>

namespace StarshipSimulator
{

/// The whole of `text` as a number, or nothing when it is empty or anything in it is not part of
/// the number (no spaces, no trailing characters). The decimal point is always '.', whatever the
/// locale. Command-line options, habitat and catalog fields all go through these.
[[nodiscard]] std::optional<int>    parseInt(std::string_view text);
[[nodiscard]] std::optional<double> parseDouble(std::string_view text);

}  // namespace StarshipSimulator
